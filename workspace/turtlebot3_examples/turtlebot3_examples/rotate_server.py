import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from turtlebot3_interfaces.srv import Rotate

# Service server that rotates the robot by a requested angle (degrees).


class RotateServer(Node):

    ANGULAR_SPEED = 0.5  # rad/s

    def __init__(self):
        super().__init__('rotate_server')
        self.srv = self.create_service(Rotate, 'rotate', self.rotate_cb)
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.current_yaw = 0.0

        self.sub = self.create_subscription(Odometry, 'odom', self.odom_cb, 10)
        self.get_logger().info('Rotate service ready')

    def odom_cb(self, msg: Odometry):
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    def rotate_cb(self, request, response):
        target_rad = math.radians(request.angle_degrees)
        target_yaw = self._normalize(self.current_yaw + target_rad)
        direction = 1.0 if target_rad >= 0 else -1.0

        self.get_logger().info(
            f'Rotating {request.angle_degrees:.1f} degrees'
        )

        twist = Twist()
        twist.angular.z = self.ANGULAR_SPEED * direction

        tolerance = 0.03  # ~1.7 degrees
        while True:
            self.pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.05)
            error = abs(self._normalize(target_yaw - self.current_yaw))
            if error < tolerance:
                break

        # Stop the robot
        self.pub.publish(Twist())

        response.success = True
        response.message = f'Rotated {request.angle_degrees:.1f} degrees'
        self.get_logger().info(response.message)
        return response

    @staticmethod
    def _normalize(angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle


def main(args=None):
    rclpy.init(args=args)
    node = RotateServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
