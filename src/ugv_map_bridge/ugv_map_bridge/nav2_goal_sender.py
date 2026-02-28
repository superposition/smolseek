"""
Nav2 Goal Sender — ROS2 node that bridges /smolseek/goal to Nav2.

Subscribes to /smolseek/goal (std_msgs/String JSON), parses target coords,
sends them to Nav2 via the /navigate_to_pose action, and publishes navigation
feedback on /smolseek/nav_status.

Non-blocking startup: polls for Nav2 action server every 2s instead of
blocking in __init__. Goals received before Nav2 is ready are rejected
with a 'failed' status.
"""

import json
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from nav2_msgs.action import NavigateToPose


GOAL_TIMEOUT_SEC = 30.0
NAV2_POLL_SEC = 2.0


class Nav2GoalSender(Node):
    def __init__(self):
        super().__init__('nav2_goal_sender')

        # --- Nav2 action client (non-blocking) ---
        self.nav2_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._nav2_ready = False

        # --- ROS2 pub/sub ---
        self.goal_sub = self.create_subscription(
            String, '/smolseek/goal', self._on_goal, 10
        )
        self.status_pub = self.create_publisher(
            String, '/smolseek/nav_status', 10
        )

        # --- State ---
        self._current_goal_handle: ClientGoalHandle | None = None
        self._current_cache_id: str | None = None
        self._goal_start_time: float | None = None

        # --- Timers ---
        self._timeout_timer = self.create_timer(1.0, self._check_timeout)
        self._nav2_poll_timer = self.create_timer(NAV2_POLL_SEC, self._poll_nav2)

        self.get_logger().info(
            'Nav2GoalSender started — listening on /smolseek/goal, '
            'polling for Nav2 action server...'
        )

    # ------------------------------------------------------------------
    # Nav2 discovery (non-blocking)
    # ------------------------------------------------------------------
    def _poll_nav2(self):
        """Periodically check if Nav2 action server is available."""
        if self._nav2_ready:
            return
        if self.nav2_client.server_is_ready():
            self._nav2_ready = True
            self.get_logger().info('Nav2 action server connected!')
            # Stop polling
            self._nav2_poll_timer.cancel()
        else:
            self.get_logger().debug('Nav2 action server not yet available...')

    # ------------------------------------------------------------------
    # Goal subscription callback
    # ------------------------------------------------------------------
    def _on_goal(self, msg: String):
        """Handle incoming goal JSON from /smolseek/goal."""
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.get_logger().error(f'Bad JSON on /smolseek/goal: {e}')
            return

        x = float(data.get('x', 0.0))
        y = float(data.get('y', 0.0))
        theta = float(data.get('theta', 0.0))
        cache_id = str(data.get('cache_id', ''))

        self.get_logger().info(
            f'Goal received — cache_id={cache_id} x={x:.2f} y={y:.2f} theta={theta:.2f}'
        )

        if not self._nav2_ready:
            self.get_logger().warn('Nav2 not ready — rejecting goal')
            self._publish_status('failed', cache_id, 0.0)
            return

        # Cancel any in-flight goal before sending a new one
        if self._current_goal_handle is not None:
            self.get_logger().warn(
                f'Cancelling previous goal (cache_id={self._current_cache_id}) for new goal'
            )
            self._current_goal_handle.cancel_goal_async()
            self._publish_status('aborted', self._current_cache_id, 0.0)

        self._current_cache_id = cache_id
        self._send_nav2_goal(x, y, theta)

    # ------------------------------------------------------------------
    # Send goal to Nav2
    # ------------------------------------------------------------------
    def _send_nav2_goal(self, x: float, y: float, theta: float):
        """Build PoseStamped and send to Nav2 navigate_to_pose action."""
        goal_msg = NavigateToPose.Goal()

        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0

        # Convert theta (yaw) to quaternion — rotation about Z axis
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = math.sin(theta / 2.0)
        pose.pose.orientation.w = math.cos(theta / 2.0)

        goal_msg.pose = pose

        self.get_logger().info(
            f'Sending NavigateToPose goal — '
            f'pos=({x:.2f}, {y:.2f}) yaw={theta:.2f} rad'
        )

        send_goal_future = self.nav2_client.send_goal_async(
            goal_msg, feedback_callback=self._on_feedback
        )
        send_goal_future.add_done_callback(self._on_goal_response)
        self._goal_start_time = time.monotonic()

    # ------------------------------------------------------------------
    # Nav2 callbacks
    # ------------------------------------------------------------------
    def _on_goal_response(self, future):
        """Called when Nav2 accepts or rejects the goal."""
        goal_handle: ClientGoalHandle = future.result()

        if not goal_handle.accepted:
            self.get_logger().warn('Nav2 rejected goal')
            self._publish_status('failed', self._current_cache_id, 0.0)
            self._reset_state()
            return

        self.get_logger().info('Nav2 accepted goal — navigating')
        self._current_goal_handle = goal_handle
        self._publish_status('navigating', self._current_cache_id, 0.0)

        # Attach result callback
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_result)

    def _on_feedback(self, feedback_msg):
        """Called periodically with Nav2 navigation feedback."""
        feedback = feedback_msg.feedback
        distance = feedback.distance_remaining

        self.get_logger().info(
            f'Nav2 feedback — distance_remaining={distance:.2f}m '
            f'(cache_id={self._current_cache_id})'
        )
        self._publish_status('navigating', self._current_cache_id, distance)

    def _on_result(self, future):
        """Called when Nav2 finishes the goal."""
        result = future.result()
        status = result.status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(
                f'Navigation succeeded — cache_id={self._current_cache_id}'
            )
            self._publish_status('arrived', self._current_cache_id, 0.0)
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warn(
                f'Navigation canceled — cache_id={self._current_cache_id}'
            )
            self._publish_status('aborted', self._current_cache_id, 0.0)
        else:
            self.get_logger().error(
                f'Navigation failed with status {status} — '
                f'cache_id={self._current_cache_id}'
            )
            self._publish_status('failed', self._current_cache_id, 0.0)

        self._reset_state()

    # ------------------------------------------------------------------
    # Timeout check
    # ------------------------------------------------------------------
    def _check_timeout(self):
        """Timer callback — abort goal if it exceeds GOAL_TIMEOUT_SEC."""
        if self._goal_start_time is None or self._current_goal_handle is None:
            return

        elapsed = time.monotonic() - self._goal_start_time
        if elapsed >= GOAL_TIMEOUT_SEC:
            self.get_logger().warn(
                f'Goal timed out after {elapsed:.1f}s — '
                f'cache_id={self._current_cache_id}'
            )
            self._current_goal_handle.cancel_goal_async()
            self._publish_status('timeout', self._current_cache_id, 0.0)
            self._reset_state()

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------
    def _publish_status(self, status: str, cache_id: str | None, distance: float):
        """Publish JSON status message on /smolseek/nav_status."""
        payload = json.dumps({
            'status': status,
            'cache_id': cache_id or '',
            'distance_remaining': round(distance, 3),
        })
        msg = String()
        msg.data = payload
        self.status_pub.publish(msg)
        self.get_logger().info(f'Published nav_status: {payload}')

    def _reset_state(self):
        """Clear in-flight goal state."""
        self._current_goal_handle = None
        self._current_cache_id = None
        self._goal_start_time = None


def main(args=None):
    rclpy.init(args=args)
    node = Nav2GoalSender()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down Nav2GoalSender')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
