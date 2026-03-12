#!/usr/bin/env python3

from __future__ import annotations

from typing import Dict

import rclpy
from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from std_msgs.msg import Bool


def _default_result(message: str, error_code: int = 0) -> FollowJointTrajectory.Result:
    result = FollowJointTrajectory.Result()
    result.error_code = error_code
    result.error_string = message
    return result


class TrajectoryGuardNode(Node):
    def __init__(self):
        super().__init__('trajectory_guard_node')

        cb_group = ReentrantCallbackGroup()
        self.declare_parameter('source_action', '/arm_controller/follow_joint_trajectory')
        self.declare_parameter('relay_action', '/arm_controller/hardening/follow_joint_trajectory')
        self.declare_parameter('safety_topic', '/arctos/hardening/ok_to_execute')
        self.declare_parameter('require_safety', True)

        self.source_action = self.get_parameter('source_action').get_parameter_value().string_value
        self.relay_action = self.get_parameter('relay_action').get_parameter_value().string_value
        self.safety_topic = self.get_parameter('safety_topic').get_parameter_value().string_value
        self.require_safety = self.get_parameter('require_safety').value

        self._safe_to_execute = False
        self._client_goals: Dict[int, object] = {}

        self.create_subscription(
            Bool,
            self.safety_topic,
            self._on_safety,
            10,
            callback_group=cb_group,
        )

        self._server = ActionServer(
            self,
            FollowJointTrajectory,
            self.relay_action,
            execute_callback=self._execute_cb,
            callback_group=cb_group,
            goal_callback=self._goal_cb,
            cancel_callback=self._cancel_cb,
        )
        self._client = ActionClient(
            self,
            FollowJointTrajectory,
            self.source_action,
            callback_group=cb_group,
        )

        self.get_logger().info(
            f'hardening action relay: {self.relay_action} -> {self.source_action}, '
            f'safety={self.safety_topic}, require_safety={self.require_safety}'
        )

    def _on_safety(self, msg: Bool):
        self._safe_to_execute = bool(msg.data)

    def _goal_cb(self, _goal_request):
        if self.require_safety and not self._safe_to_execute:
            self.get_logger().warn('Rejecting trajectory goal: hardening blocked (ok_to_execute=false)')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    async def _execute_cb(self, server_goal_handle):
        if not self._safe_to_execute and self.require_safety:
            server_goal_handle.abort()
            return _default_result('trajectory blocked by hardening')

        try:
            if not self._client.server_is_ready():
                for _ in range(20):
                    if self._client.server_is_ready():
                        break
                    rclpy.spin_once(self, timeout_sec=0.1)
                else:
                    server_goal_handle.abort()
                    return _default_result('upstream follow_joint_trajectory server unavailable')

            send_goal_future = self._client.send_goal_async(
                server_goal_handle.request,
                feedback_callback=lambda fb: self._on_client_feedback(server_goal_handle, fb),
            )
            client_goal_handle = await send_goal_future
            if not client_goal_handle.accepted:
                server_goal_handle.abort()
                return _default_result('upstream server rejected goal')

            self._client_goals[id(server_goal_handle)] = client_goal_handle
            try:
                result_future = client_goal_handle.get_result_async()
                result_response = await result_future
                relay_result = result_response.result

                if result_response.status == GoalStatus.STATUS_SUCCEEDED:
                    server_goal_handle.succeed()
                else:
                    server_goal_handle.abort()
                return relay_result
            finally:
                self._client_goals.pop(id(server_goal_handle), None)
        except Exception as exc:
            self.get_logger().error(f'hardening relay failed: {exc}')
            server_goal_handle.abort()
            return _default_result(f'hardening relay failed: {exc}')

    def _on_client_feedback(self, server_goal_handle, feedback):
        server_goal_handle.publish_feedback(feedback.feedback)

    def _cancel_cb(self, goal_handle):
        client_goal_handle = self._client_goals.pop(id(goal_handle), None)
        if client_goal_handle is not None:
            client_goal_handle.cancel_goal_async()
        return CancelResponse.ACCEPT


def main(argv=None):
    rclpy.init(args=argv)
    node = TrajectoryGuardNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
