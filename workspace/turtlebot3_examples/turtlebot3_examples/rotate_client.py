import sys

import rclpy
from rclpy.node import Node
from turtlebot3_interfaces.srv import Rotate

# Service client that sends a rotation request to the rotate service.


class RotateClient(Node):

    def __init__(self):
        super().__init__('rotate_client')
        self.client = self.create_client(Rotate, 'rotate')

        self.declare_parameter('angle', 90.0)

        while not self.client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('Waiting for rotate service...')

    def send_request(self):
        angle = self.get_parameter('angle').get_parameter_value().double_value
        req = Rotate.Request()
        req.angle_degrees = angle
        self.get_logger().info(f'Requesting rotation: {angle} degrees')

        future = self.client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()


def main(args=None):
    rclpy.init(args=args)
    node = RotateClient()

    result = node.send_request()
    if result is not None:
        node.get_logger().info(f'Result: success={result.success}, {result.message}')
    else:
        node.get_logger().error('Service call failed')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
