import rclpy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped


def main():
    rclpy.init()
    node = rclpy.create_node('cmd_vel_relay')

    publisher = node.create_publisher(TwistStamped, '/diff_drive_base/cmd_vel', 10)

    def callback(msg: Twist):
        stamped = TwistStamped()
        stamped.header.stamp = node.get_clock().now().to_msg()
        stamped.header.frame_id = ''
        stamped.twist = msg
        publisher.publish(stamped)

    node.create_subscription(Twist, '/cmd_vel', callback, 10)
    node.get_logger().info('cmd_vel_relay started: relaying /cmd_vel -> /diff_drive_base/cmd_vel')

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
