// forklift_position_pid.cpp
//
// Software PID position controller for the forklift prismatic joint.
//
// Why this exists:
//   gz_ros2_control's "position" command interface is implemented internally
//   as velocity-tracking — it commands a joint VELOCITY proportional to
//   position error, not effort. A vertical prismatic joint lifting real mass
//   against gravity cannot be held or driven through that interface
//   (gz_ros2_control issue #192). The fix is to use the "effort" command
//   interface instead and close the position loop in software here, where
//   we can apply actual force to fight gravity.
//
// PAYLOAD-AWARE UPDATE:
//   The original version hardcoded gravity feedforward and PID gains for the
//   empty forklift mass (6.0 kg). That's fine for an empty fork, but with a
//   real payload on top (a pallet, a box of masks, eventually up to ~200 kg)
//   the fixed 58.86 N feedforward is nowhere near enough to hold the load,
//   and the fixed kp/kd were tuned for a much lighter system — so the fork
//   just sags to the bottom of its 0.20 m travel and sits there, looking
//   "static" even though nothing is actually wrong with the model.
//
//   Fix: gravity feedforward and PID gains are now recomputed from a live
//   `payload_mass_kg` parameter, using a fixed natural frequency / damping
//   ratio design (critically damped 2nd-order system by default):
//
//       total_mass = forklift_mass_kg + payload_mass_kg
//       kp = total_mass * wn^2
//       kd = 2 * zeta * total_mass * wn
//       gravity_ff = total_mass * 9.81
//
//   Set the payload mass BEFORE lifting, e.g. from your pick-and-place
//   task logic:
//       ros2 param set /forklift_position_pid payload_mass_kg 200.0
//   or programmatically via rclcpp::AsyncParametersClient. Gains and
//   feedforward recompute automatically the moment the parameter changes.
//
// Architecture (unchanged from before):
//   /forklift_controller/commands          (Float64MultiArray, external API)
//        ▼
//   forklift_position_pid (this node)      — software PID, 50 Hz
//        ▼
//   /forklift_effort_controller/commands   (Float64MultiArray)
//        ▼
//   effort_controllers/JointGroupEffortController  (ros2_control)
//        ▼
//   prismatic_forklift joint (Gazebo applies effort directly as force)

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <algorithm>
#include <atomic>
#include <mutex>

class ForkliftPositionPid : public rclcpp::Node
{
public:
  ForkliftPositionPid() : Node("forklift_position_pid")
  {
    // ── Tunable parameters (all live-updatable via ros2 param set) ──

    // Physical / design parameters used to DERIVE kp, kd, gravity_ff.
    // These replace hardcoded gain values from the previous version.
    declare_parameter<double>("forklift_mass_kg",       6.0);   // from URDF <inertial> for "forklift"
    declare_parameter<double>("payload_mass_kg",         0.0);
    declare_parameter<double>("natural_freq_rad_s",      4.0);   // wn — same responsiveness as before
    declare_parameter<double>("damping_ratio",           1.0);   // zeta = 1.0 -> critically damped, no overshoot
    declare_parameter<double>("gravity_mps2",            9.81);

    // Everything below this line is unchanged in spirit from the original.
    declare_parameter<double>("ki",                      0.0);
    declare_parameter<double>("max_effort",              5000.0);
    declare_parameter<double>("min_effort",             -5000.0);
    declare_parameter<double>("min_position",            0.0);
    declare_parameter<double>("max_position",            0.20);
    declare_parameter<double>("integral_clamp",          50.0);
    declare_parameter<double>("setpoint_slew_m_per_s",   0.5);    // matches URDF <limit velocity>
    declare_parameter<double>("deadband_m",              0.001);  // 1 mm — stop adjusting inside this
    declare_parameter<std::string>("joint_name",         "prismatic_forklift");

    // Snapshot live params into cached fields for fast access from timer
    refresh_params();
    recompute_gains_for_load();

    joint_name_ = get_parameter("joint_name").as_string();

    // Start at the bottom (0.0) — matches your forklift's resting/loaded position.
    target_setpoint_ = min_pos_;
    setpoint_ = min_pos_;

    // Parameter callback — re-snapshot AND re-derive gains when any param changes.
    // This is what makes payload_mass_kg live-settable: the moment it changes,
    // kp/kd/gravity_ff are recomputed for the new total mass on the next
    // control_step() tick.
    param_cb_handle_ = add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter>& /*params*/) {
        params_dirty_.store(true);
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        return result;
      });

    // ── Subscriptions ──
    cmd_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      "/forklift_controller/commands", 10,
      [this](std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        if (msg->data.empty()) return;
        std::lock_guard<std::mutex> lk(state_mutex_);
        target_setpoint_ = std::clamp(msg->data[0], min_pos_, max_pos_);
        integral_ = 0.0;  // reset windup on new goal
        RCLCPP_INFO(get_logger(),
          "Forklift target: %.3f m (slewing from %.3f at %.2f m/s, payload=%.1f kg)",
          target_setpoint_, setpoint_, slew_rate_, payload_mass_kg_);
      });

    state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::JointState::SharedPtr msg) {
        for (size_t i = 0; i < msg->name.size(); ++i) {
          if (msg->name[i] == joint_name_) {
            std::lock_guard<std::mutex> lk(state_mutex_);
            current_pos_ = msg->position[i];
            current_vel_ = (i < msg->velocity.size()) ? msg->velocity[i] : 0.0;
            have_state_ = true;
            return;
          }
        }
      });

    // ── Publisher ──
    effort_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      "/forklift_effort_controller/commands", 10);

    // ── Control loop at 50 Hz ──
    last_time_ = now();
    timer_ = create_wall_timer(
      std::chrono::milliseconds(20),
      [this]() { control_step(); });

    RCLCPP_INFO(get_logger(),
      "Forklift PID ready. forklift_mass=%.1fkg payload=%.1fkg -> kp=%.1f kd=%.1f ff=%.1fN",
      forklift_mass_kg_, payload_mass_kg_, kp_, kd_, gravity_ff_);
    RCLCPP_INFO(get_logger(),
      "Live tune: ros2 param set /forklift_position_pid payload_mass_kg <value>");
  }

private:
  void refresh_params()
  {
    forklift_mass_kg_ = get_parameter("forklift_mass_kg").as_double();
    payload_mass_kg_  = get_parameter("payload_mass_kg").as_double();
    natural_freq_     = get_parameter("natural_freq_rad_s").as_double();
    damping_ratio_    = get_parameter("damping_ratio").as_double();
    gravity_mps2_     = get_parameter("gravity_mps2").as_double();

    ki_           = get_parameter("ki").as_double();
    max_effort_   = get_parameter("max_effort").as_double();
    min_effort_   = get_parameter("min_effort").as_double();
    min_pos_      = get_parameter("min_position").as_double();
    max_pos_      = get_parameter("max_position").as_double();
    i_clamp_      = get_parameter("integral_clamp").as_double();
    slew_rate_    = get_parameter("setpoint_slew_m_per_s").as_double();
    deadband_     = get_parameter("deadband_m").as_double();
  }

  // Re-derive kp, kd, gravity_ff from current total mass (forklift + payload).
  // This is the core of the payload-aware fix. Called once at startup and
  // again any time refresh_params() runs after a parameter change.
  void recompute_gains_for_load()
  {
    double total_mass = std::max(0.0, forklift_mass_kg_ + payload_mass_kg_);
    double wn = natural_freq_;

    kp_ = total_mass * wn * wn;
    kd_ = 2.0 * damping_ratio_ * total_mass * wn;
    gravity_ff_ = total_mass * gravity_mps2_;
  }

  void control_step()
  {
    if (params_dirty_.exchange(false)) {
      refresh_params();
      recompute_gains_for_load();
      RCLCPP_INFO(get_logger(),
        "Params updated: forklift_mass=%.1fkg payload=%.1fkg -> kp=%.1f kd=%.1f ff=%.1fN",
        forklift_mass_kg_, payload_mass_kg_, kp_, kd_, gravity_ff_);
    }

    if (!have_state_) return;

    auto t = now();
    double dt = (t - last_time_).seconds();
    last_time_ = t;
    if (dt <= 0.0 || dt > 0.5) dt = 0.02;  // sanity bounds

    double pos, vel, target;
    {
      std::lock_guard<std::mutex> lk(state_mutex_);
      pos = current_pos_;
      vel = current_vel_;
      target = target_setpoint_;
    }

    // ── 1. Setpoint slew limit ──
    double max_step = slew_rate_ * dt;
    double diff_to_target = target - setpoint_;
    if (std::abs(diff_to_target) <= max_step) {
      setpoint_ = target;
    } else {
      setpoint_ += (diff_to_target > 0 ? max_step : -max_step);
    }

    // ── 2. PID computation ──
    double error = setpoint_ - pos;

    bool in_deadband = (std::abs(error) < deadband_) &&
                       (std::abs(target - pos) < deadband_) &&
                       (std::abs(vel) < 0.005);

    if (in_deadband) {
      // Hold position with just the gravity feedforward — now scaled to the
      // ACTUAL total mass (forklift + payload), so a 200 kg pallet gets
      // ~2021 N of holding force instead of the old fixed ~59 N.
      integral_ = 0.0;
      double effort = std::clamp(gravity_ff_, min_effort_, max_effort_);
      publish_effort(effort);
      return;
    }

    integral_ += error * dt;
    integral_ = std::clamp(integral_, -i_clamp_, i_clamp_);

    double derivative = -vel;  // slewed setpoint changes slowly, so d(err)/dt ≈ -vel

    double effort = kp_ * error
                  + ki_ * integral_
                  + kd_ * derivative
                  + gravity_ff_;

    effort = std::clamp(effort, min_effort_, max_effort_);
    publish_effort(effort);
  }

  void publish_effort(double effort)
  {
    std_msgs::msg::Float64MultiArray msg;
    msg.data = {effort};
    effort_pub_->publish(msg);
  }

  // Physical / design parameters (feed recompute_gains_for_load)
  double forklift_mass_kg_{6.0};
  double payload_mass_kg_{0.0};
  double natural_freq_{4.0};
  double damping_ratio_{1.0};
  double gravity_mps2_{9.81};

  // Derived gains (recomputed, not fixed constants anymore)
  double kp_{96.0}, kd_{48.0};
  double gravity_ff_{58.86};

  double ki_{0.0};
  double max_effort_{5000.0}, min_effort_{-5000.0};
  double min_pos_{0.0}, max_pos_{0.20};
  double i_clamp_{50.0}, slew_rate_{0.5}, deadband_{0.001};

  std::string joint_name_;
  std::atomic<bool> params_dirty_{false};

  std::mutex state_mutex_;
  double setpoint_{0.0};
  double target_setpoint_{0.0};
  double current_pos_{0.0}, current_vel_{0.0};
  double integral_{0.0};
  bool have_state_{false};

  rclcpp::Time last_time_;

  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr cmd_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr     state_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr    effort_pub_;
  rclcpp::TimerBase::SharedPtr                                      timer_;
  OnSetParametersCallbackHandle::SharedPtr                          param_cb_handle_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ForkliftPositionPid>());
  rclcpp::shutdown();
  return 0;
}