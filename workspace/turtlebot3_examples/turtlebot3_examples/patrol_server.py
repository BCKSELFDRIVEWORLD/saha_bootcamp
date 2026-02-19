import math
import time

import rclpy
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from turtlebot3_interfaces.action import Patrol

# Action server that navigates the robot through a list of waypoints,
# publishing feedback (current waypoint index, distance remaining) and
# returning total distance travelled on completion.


class PatrolServer(Node):

    LINEAR_SPEED = 0.2    # m/s
    ANGULAR_SPEED = 0.5   # rad/s
    GOAL_TOL = 0.15       # metres
    ANGLE_TOL = 0.1       # radians

    def __init__(self):
        super().__init__('patrol_server')
        self.cb_group = ReentrantCallbackGroup()

        self._action_server = ActionServer(
            self,
            Patrol,
            'patrol',
            execute_callback=self.execute_cb,
            goal_callback=self.goal_cb,
            cancel_callback=self.cancel_cb,
            callback_group=self.cb_group,
        )

        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.sub = self.create_subscription(
            Odometry, 'odom', self.odom_cb, 10,
            callback_group=self.cb_group,
        )
        self.get_logger().info('Patrol action server ready')

    def odom_cb(self, msg: Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)

    def goal_cb(self, goal_request):
        if len(goal_request.waypoints) == 0:
            self.get_logger().warn('Empty waypoint list, rejecting')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def cancel_cb(self, goal_handle):
        self.get_logger().info('Cancel requested')
        return CancelResponse.ACCEPT

    async def execute_cb(self, goal_handle):
        waypoints = goal_handle.request.waypoints
        self.get_logger().info(f'Patrol started with {len(waypoints)} waypoints')

        total_distance = 0.0
        feedback = Patrol.Feedback()

        for i, wp in enumerate(waypoints):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.pub.publish(Twist())
                result = Patrol.Result()
                result.success = False
                result.total_distance = total_distance
                return result

            self.get_logger().info(f'Heading to waypoint {i}: ({wp.x:.2f}, {wp.y:.2f})')

            prev_x, prev_y = self.x, self.y

            # Rotate towards waypoint
            self._rotate_towards(wp.x, wp.y)

            # Drive towards waypoint
            while True:
                if goal_handle.is_cancel_requested:
                    break

                dx = wp.x - self.x
                dy = wp.y - self.y
                dist = math.hypot(dx, dy)

                if dist < self.GOAL_TOL:
                    break

                # Publish feedback
                feedback.current_wp = i
                feedback.distance_remaining = dist
                goal_handle.publish_feedback(feedback)

                # Small heading corrections while driving
                desired_yaw = math.atan2(dy, dx)
                yaw_err = self._normalize(desired_yaw - self.yaw)

                twist = Twist()
                twist.linear.x = self.LINEAR_SPEED
                twist.angular.z = max(-self.ANGULAR_SPEED,
                                      min(self.ANGULAR_SPEED, yaw_err * 2.0))
                self.pub.publish(twist)
                time.sleep(0.1)

            # Accumulate distance for this segment
            seg_dist = math.hypot(self.x - prev_x, self.y - prev_y)
            total_distance += seg_dist

        # Stop the robot
        self.pub.publish(Twist())

        result = Patrol.Result()
        result.success = True
        result.total_distance = total_distance
        goal_handle.succeed()
        self.get_logger().info(f'Patrol done, total distance: {total_distance:.2f} m')
        return result

    def _rotate_towards(self, tx, ty):
        desired = math.atan2(ty - self.y, tx - self.x)
        twist = Twist()
        for _ in range(200):  # timeout ~20 s
            err = self._normalize(desired - self.yaw)
            if abs(err) < self.ANGLE_TOL:
                break
            twist.angular.z = self.ANGULAR_SPEED if err > 0 else -self.ANGULAR_SPEED
            self.pub.publish(twist)
            time.sleep(0.1)
        self.pub.publish(Twist())

    @staticmethod
    def _normalize(angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle


def main(args=None):
    rclpy.init(args=args)
    node = PatrolServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
