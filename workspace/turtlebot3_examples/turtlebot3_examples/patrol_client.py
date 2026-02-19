import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import Point
from turtlebot3_interfaces.action import Patrol

# Action client that sends a list of waypoints to the patrol action server
# and prints feedback as the robot navigates.

DEFAULT_WAYPOINTS = [
    (1.0, 0.0),
    (1.0, 1.0),
    (0.0, 1.0),
    (0.0, 0.0),
]


class PatrolClient(Node):

    def __init__(self):
        super().__init__('patrol_client')
        self._client = ActionClient(self, Patrol, 'patrol')
        self.get_logger().info('Waiting for patrol action server...')
        self._client.wait_for_server()
        self.get_logger().info('Patrol action server connected')

    def send_goal(self):
        goal = Patrol.Goal()
        for x, y in DEFAULT_WAYPOINTS:
            p = Point()
            p.x = x
            p.y = y
            p.z = 0.0
            goal.waypoints.append(p)

        self.get_logger().info(
            f'Sending patrol with {len(goal.waypoints)} waypoints'
        )

        future = self._client.send_goal_async(
            goal, feedback_callback=self.feedback_cb
        )
        future.add_done_callback(self.goal_response_cb)

    def goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            return

        self.get_logger().info('Goal accepted')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_cb)

    def feedback_cb(self, feedback_msg):
        fb = feedback_msg.feedback
        self.get_logger().info(
            f'WP {fb.current_wp} | distance remaining: {fb.distance_remaining:.2f} m'
        )

    def result_cb(self, future):
        result = future.result().result
        if result.success:
            self.get_logger().info(
                f'Patrol complete! Total distance: {result.total_distance:.2f} m'
            )
        else:
            self.get_logger().warn('Patrol was cancelled or failed')
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = PatrolClient()
    node.send_goal()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()


if __name__ == '__main__':
    main()
