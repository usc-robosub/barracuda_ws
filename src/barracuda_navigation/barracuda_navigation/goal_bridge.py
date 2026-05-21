#!/usr/bin/env python3
"""Bridge /target_pose PoseStamped → Nav2 NavigateToPose action."""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose


class GoalBridge(Node):
    def __init__(self):
        super().__init__("goal_bridge")
        self._action_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self._sub = self.create_subscription(
            PoseStamped, "/target_pose", self._on_target_pose, 10
        )
        self.get_logger().info("GoalBridge ready — listening on /target_pose")

    def _on_target_pose(self, msg: PoseStamped):
        if not self._action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn("navigate_to_pose action server not available, skipping goal")
            return
        goal = NavigateToPose.Goal()
        goal.pose = msg
        self.get_logger().info(
            f"Sending goal → x={msg.pose.position.x:.2f} y={msg.pose.position.y:.2f}"
        )
        self._action_client.send_goal_async(
            goal,
            feedback_callback=self._feedback_cb,
        ).add_done_callback(self._goal_response_cb)

    def _goal_response_cb(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warn("Goal rejected by bt_navigator")
            return
        self.get_logger().info("Goal accepted")
        handle.get_result_async().add_done_callback(self._result_cb)

    def _result_cb(self, future):
        result = future.result()
        self.get_logger().info(f"Navigation completed with result: {result.status}")

    def _feedback_cb(self, feedback):
        fb = feedback.feedback
        self.get_logger().debug(
            f"Distance remaining: {fb.distance_remaining:.2f} m"
        )


def main():
    rclpy.init()
    node = GoalBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
