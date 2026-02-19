import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

# Subscribe to /odom and print the robot's x, y position and yaw angle.


class OdomListener(Node):

    def __init__(self):
        super().__init__('odom_listener')
        self.sub = self.create_subscription(Odometry, 'odom', self.odom_cb, 10)
        self.get_logger().info('Odom listener started')

    def odom_cb(self, msg: Odometry):
        pos = msg.pose.pose.position
        q = msg.pose.pose.orientation
        # Convert quaternion to yaw
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        self.get_logger().info(
            f'x={pos.x:.3f}  y={pos.y:.3f}  yaw={math.degrees(yaw):.1f} deg'
        )


def main(args=None):
    rclpy.init(args=args)
    node = OdomListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
