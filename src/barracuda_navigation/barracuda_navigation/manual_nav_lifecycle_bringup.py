#!/usr/bin/env python3
"""Manually configure and activate Nav2 lifecycle nodes for replay."""

from __future__ import annotations

import time

import rclpy
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState, GetState
from rclpy.node import Node


class ManualNavLifecycleBringup(Node):
    def __init__(self) -> None:
        super().__init__("manual_nav_lifecycle_bringup")
        self.declare_parameter(
            "node_names",
            ["planner_server", "controller_server", "behavior_server", "bt_navigator"],
        )
        self.declare_parameter("service_wait_sec", 60.0)
        self.declare_parameter("poll_period_sec", 0.5)

        self._node_names = list(self.get_parameter("node_names").value)
        self._service_wait_sec = float(self.get_parameter("service_wait_sec").value)
        self._poll_period_sec = float(self.get_parameter("poll_period_sec").value)

    def run(self) -> int:
        self.get_logger().info(
            f"Waiting for lifecycle services on: {', '.join(self._node_names)}"
        )
        deadline = time.monotonic() + self._service_wait_sec
        for node_name in self._node_names:
            if not self._wait_for_service(f"/{node_name}/get_state", deadline):
                return 1
            if not self._wait_for_service(f"/{node_name}/change_state", deadline):
                return 1

        for node_name in self._node_names:
            if not self._change_state(
                node_name, Transition.TRANSITION_CONFIGURE, "configure"
            ):
                return 1

        for node_name in self._node_names:
            if not self._change_state(
                node_name, Transition.TRANSITION_ACTIVATE, "activate"
            ):
                return 1

        self.get_logger().info("Manual Nav2 lifecycle bringup completed.")
        return 0

    def _wait_for_service(self, service_name: str, deadline: float) -> bool:
        client = self.create_client(GetState, service_name)
        while time.monotonic() < deadline:
            if client.wait_for_service(timeout_sec=self._poll_period_sec):
                return True
        self.get_logger().error(f"Timed out waiting for service {service_name}")
        return False

    def _change_state(self, node_name: str, transition_id: int, label: str) -> bool:
        change_client = self.create_client(ChangeState, f"/{node_name}/change_state")
        get_client = self.create_client(GetState, f"/{node_name}/get_state")

        request = ChangeState.Request()
        request.transition.id = transition_id
        future = change_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=self._service_wait_sec)
        if not future.done() or future.result() is None or not future.result().success:
            self.get_logger().error(f"Failed to {label} {node_name}")
            return False

        state_future = get_client.call_async(GetState.Request())
        rclpy.spin_until_future_complete(
            self, state_future, timeout_sec=self._service_wait_sec
        )
        if not state_future.done() or state_future.result() is None:
            self.get_logger().error(f"Failed to read state after {label} on {node_name}")
            return False

        self.get_logger().info(
            f"{node_name} after {label}: {state_future.result().current_state.label}"
        )
        return True


def main() -> None:
    rclpy.init()
    node = ManualNavLifecycleBringup()
    try:
        raise SystemExit(node.run())
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
