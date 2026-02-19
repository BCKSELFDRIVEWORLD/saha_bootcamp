import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

# Drive the TurtleBot3 in a square pattern by alternating between
# straight segments and 90-degree turns.


class SquareDriver(Node):

    SIDE_LENGTH = 1.0       # metres per side
    LINEAR_SPEED = 0.2      # m/s
    ANGULAR_SPEED = 0.5     # rad/s
    TURN_ANGLE = 1.5708     # ~90 degrees in radians

    def __init__(self):
        super().__init__('square_driver')

        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.state = 'forward'   # 'forward' or 'turn'
        self.distance = 0.0
        self.angle = 0.0
        self.side_count = 0

        self.get_logger().info('Square driver started')

    def timer_callback(self):
        msg = Twist()
        dt = 0.1

        if self.state == 'forward':
            msg.linear.x = self.LINEAR_SPEED
            self.distance += self.LINEAR_SPEED * dt
            if self.distance >= self.SIDE_LENGTH:
                self.distance = 0.0
                self.state = 'turn'
                self.get_logger().info(f'Side {self.side_count + 1} done, turning')

        elif self.state == 'turn':
            msg.angular.z = self.ANGULAR_SPEED
            self.angle += self.ANGULAR_SPEED * dt
            if self.angle >= self.TURN_ANGLE:
                self.angle = 0.0
                self.side_count += 1
                self.state = 'forward'
                if self.side_count >= 4:
                    self.side_count = 0
                    self.get_logger().info('Square complete, starting next lap')

        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SquareDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop the robot before shutting down
        stop_msg = Twist()
        node.pub.publish(stop_msg)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
