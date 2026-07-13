#!/usr/bin/env python3

import sys
import time

import rclpy
from rclpy.node import Node

from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType

from std_msgs.msg import Float64MultiArray


# ============================================================
# Configuration
# ============================================================

PICKUP_MASS = 100.0
PLACE_MASS = 0.0

PICKUP_HEIGHT = 0.09
PLACE_HEIGHT = 0.0

WAIT_AFTER_PICK = 3.0
WAIT_AFTER_PLACE = 3.0


# ============================================================
# Mission Helper Node
# ============================================================

class MissionHelper(Node):

    def __init__(self):
        super().__init__("mission_helper")

        # Publisher for forklift height
        self.fork_pub = self.create_publisher(
            Float64MultiArray,
            "/forklift_controller/commands",
            10
        )

        # Client to change payload parameter
        self.param_client = self.create_client(
            SetParameters,
            "/forklift_position_pid/set_parameters"
        )

        self.get_logger().info("Waiting for parameter service...")

        while not self.param_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Still waiting...")

        self.get_logger().info("Parameter service connected.")

    # --------------------------------------------------------
    # Set payload mass
    # --------------------------------------------------------

    def set_payload_mass(self, mass):

        request = SetParameters.Request()

        param = Parameter()
        param.name = "payload_mass_kg"

        value = ParameterValue()
        value.type = ParameterType.PARAMETER_DOUBLE
        value.double_value = mass

        param.value = value

        request.parameters.append(param)

        future = self.param_client.call_async(request)

        rclpy.spin_until_future_complete(self, future)

        if future.result() is None:
            self.get_logger().error("Failed to set payload mass.")
            return False

        self.get_logger().info(f"Payload mass set to {mass:.1f} kg")

        return True

    # --------------------------------------------------------
    # Publish forklift height
    # --------------------------------------------------------

    def move_forklift(self, height):

        msg = Float64MultiArray()

        msg.data = [height]

        self.fork_pub.publish(msg)

        self.get_logger().info(
            f"Forklift command sent : {height:.3f} m"
        )

    # --------------------------------------------------------
    # PICK
    # --------------------------------------------------------

    def pick(self):

        self.get_logger().info("========== PICK ==========")

        # Step 1 : Set payload mass
        if not self.set_payload_mass(PICKUP_MASS):
            return

        time.sleep(0.5)

        # Step 2 : Raise forklift
        self.move_forklift(PICKUP_HEIGHT)

        time.sleep(WAIT_AFTER_PICK)

        self.get_logger().info("Pick sequence completed.")

    # --------------------------------------------------------
    # PLACE
    # --------------------------------------------------------

    def place(self):

        self.get_logger().info("========== PLACE ==========")

        # Step 1 : Remove payload
        if not self.set_payload_mass(PLACE_MASS):
            return

        time.sleep(0.5)

        # Step 2 : Lower forklift
        self.move_forklift(PLACE_HEIGHT)

        time.sleep(WAIT_AFTER_PLACE)

        self.get_logger().info("Place sequence completed.")

# ============================================================
# Main
# ============================================================


def main(args=None):

    rclpy.init(args=args)

    node = MissionHelper()

    if len(sys.argv) < 2:
        node.get_logger().error(
            "Usage:\n"
            "  ros2 run palletizing_amr mission_helper pick\n"
            "  ros2 run palletizing_amr mission_helper place"
        )
        node.destroy_node()
        rclpy.shutdown()
        return

    command = sys.argv[1].lower()

    if command == "pick":
        node.pick()

    elif command == "place":
        node.place()

    else:
        node.get_logger().error(
            f"Unknown command '{command}'. "
            "Use 'pick' or 'place'."
        )

    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
