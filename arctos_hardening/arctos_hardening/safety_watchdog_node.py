#!/usr/bin/env python3

import math
from typing import List, Set

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64, Int32
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.time import Time
import tf2_ros


def _to_float_list(values):
    return [float(v) for v in values] if values is not None else []


class SafetyWatchdogNode(Node):
    def __init__(self):
        super().__init__('safety_watchdog_node')

        self.declare_parameter('joint_limit_margin_rad', [0.0] * 6)
        self.declare_parameter(
            'joint_lower_limits',
            [-3.14, -1.5, -2.2, -2.2, -1.8, -3.14],
        )
        self.declare_parameter(
            'joint_upper_limits',
            [3.14, 1.5, 2.2, 2.2, 1.8, 3.14],
        )
        self.declare_parameter('max_execution_error', 0.08)
        self.declare_parameter('plan_status_topic', '/move_group/planning_result')
        self.declare_parameter('execution_error_topic', '/arctos/execution/error')
        self.declare_parameter('require_execution_error', False)
        self.declare_parameter('planning_fail_warn_count', 2)
        self.declare_parameter('planning_fail_error_count', 5)
        self.declare_parameter('joint_state_topic', '/joint_states')
        self.declare_parameter('ok_to_execute_topic', '/arctos/hardening/ok_to_execute')
        self.declare_parameter('diagnostic_topic', '/arctos/hardening/safety')

        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('required_frames', ['tool0'])
        self.declare_parameter('warning_age_s', 0.10)
        self.declare_parameter('error_age_s', 0.30)
        self.declare_parameter('required_update_hz', 10.0)
        self.declare_parameter('watchdog_period_ms', 50.0)

        self.joint_limit_margin_rad = _to_float_list(self.get_parameter('joint_limit_margin_rad').value)
        self.joint_lower_limits = _to_float_list(self.get_parameter('joint_lower_limits').value)
        self.joint_upper_limits = _to_float_list(self.get_parameter('joint_upper_limits').value)
        self.max_execution_error = float(self.get_parameter('max_execution_error').value)
        self.require_execution_error = bool(self.get_parameter('require_execution_error').value)
        self.planning_warn_count = int(self.get_parameter('planning_fail_warn_count').value)
        self.planning_error_count = int(self.get_parameter('planning_fail_error_count').value)
        self.required_frames = list(self.get_parameter('required_frames').value)
        self.warning_age = float(self.get_parameter('warning_age_s').value)
        self.error_age = float(self.get_parameter('error_age_s').value)
        self.required_hz = float(self.get_parameter('required_update_hz').value)
        self.watchdog_period = float(self.get_parameter('watchdog_period_ms').value) / 1000.0
        self._joint_topics: Set[str] = {
            '/joint_states',
            '/arctos/joint_states',
            self.get_parameter('joint_state_topic').value,
        }

        self._latest_joint = None
        self._latest_exec_error = None
        self._planning_fail_streak = 0

        for topic in sorted(self._joint_topics):
            self.create_subscription(
                JointState,
                topic,
                self._on_joint_state,
                10,
            )
        self.create_subscription(
            Int32,
            self.get_parameter('plan_status_topic').value,
            self._on_plan_status,
            10,
        )
        if self.require_execution_error:
            self.create_subscription(
                Float64,
                self.get_parameter('execution_error_topic').value,
                self._on_exec_error,
                10,
            )

        self.ok_pub = self.create_publisher(
            Bool,
            self.get_parameter('ok_to_execute_topic').value,
            10,
        )
        self.diag_pub = self.create_publisher(
            DiagnosticArray,
            self.get_parameter('diagnostic_topic').value,
            10,
        )

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.timer = self.create_timer(self.watchdog_period, self._evaluate)

    def _on_joint_state(self, msg: JointState):
        self._latest_joint = msg

    def _on_plan_status(self, msg: Int32):
        if msg.data == 0:
            self._planning_fail_streak = 0
        else:
            self._planning_fail_streak += 1

    def _on_exec_error(self, msg: Float64):
        self._latest_exec_error = float(msg.data)

    def _emit_status(self, status: List[DiagnosticStatus], level: int, name: str, message: str, values=None):
        if values is None:
            values = []
        status.append(DiagnosticStatus(level=level, name=name, message=message, values=values))

    def _check_joint_limits(self, status: List[DiagnosticStatus]):
        if self._latest_joint is None:
            self._emit_status(status, DiagnosticStatus.WARN, 'joint_limit_gate', 'no joint_state yet')
            return

        if not self.joint_lower_limits or not self.joint_upper_limits:
            self._emit_status(status, DiagnosticStatus.OK, 'joint_limit_gate', 'joint limits not configured')
            return

        if len(self.joint_lower_limits) != len(self.joint_upper_limits) or len(self._latest_joint.position) < len(self.joint_lower_limits):
            self._emit_status(status, DiagnosticStatus.ERROR, 'joint_limit_gate', 'joint limit array size mismatch')
            return

        for i, q in enumerate(self._latest_joint.position[:len(self.joint_lower_limits)]):
            default_margin = self.joint_limit_margin_rad[i] if i < len(self.joint_limit_margin_rad) else 0.0
            name = self._latest_joint.name[i] if i < len(self._latest_joint.name) else str(i)
            if not (self.joint_lower_limits[i] + default_margin < q < self.joint_upper_limits[i] - default_margin):
                side = 'lower' if q <= self.joint_lower_limits[i] + default_margin else 'upper'
                self._emit_status(
                    status,
                    DiagnosticStatus.WARN,
                    'joint_limit_gate',
                    f'{name} near {side} limit',
                    [KeyValue(key='q', value=str(q)), KeyValue(key='index', value=str(i))],
                )
                return

        self._emit_status(status, DiagnosticStatus.OK, 'joint_limit_gate', 'joint margins clear')

    def _check_execution_error(self, status: List[DiagnosticStatus]):
        if not self.require_execution_error:
            self._emit_status(status, DiagnosticStatus.OK, 'execution_error_gate', 'execution_error monitoring disabled')
            return

        if self._latest_exec_error is None:
            self._emit_status(status, DiagnosticStatus.WARN, 'execution_error_gate', 'no execution error sample yet')
            return

        level = DiagnosticStatus.OK
        message = 'ok'
        if abs(self._latest_exec_error) > self.max_execution_error:
            level = DiagnosticStatus.ERROR
            message = f'exceeds max_execution_error ({self.max_execution_error})'

        self._emit_status(status, level, 'execution_error_gate', message, [
            KeyValue(key='execution_error', value=f'{self._latest_exec_error:.6f}')
        ])

    def _check_planning_streak(self, status: List[DiagnosticStatus]):
        if self._planning_fail_streak >= self.planning_error_count:
            level = DiagnosticStatus.ERROR
            message = f'planning fail streak {self._planning_fail_streak}'
        elif self._planning_fail_streak >= self.planning_warn_count:
            level = DiagnosticStatus.WARN
            message = f'planning fail streak {self._planning_fail_streak}'
        else:
            level = DiagnosticStatus.OK
            message = 'planning stability normal'

        self._emit_status(
            status,
            level,
            'planning_fail_gate',
            message,
            [KeyValue(key='planning_fail_streak', value=str(self._planning_fail_streak))],
        )

    def _check_tf(self, status: List[DiagnosticStatus]):
        now = self.get_clock().now()
        base_frame = self.get_parameter('base_frame').value
        for frame in self.required_frames:
            try:
                tf = self.tf_buffer.lookup_transform(base_frame, frame, Time())
                if tf.header.stamp.sec == 0 and tf.header.stamp.nanosec == 0:
                    self._emit_status(
                        status,
                        DiagnosticStatus.OK,
                        'tf_gate',
                        f'{frame} static/no-timestamp transform',
                    )
                    continue
                age = (now - Time.from_msg(tf.header.stamp)).nanoseconds * 1e-9

                if age > self.error_age:
                    level = DiagnosticStatus.ERROR
                    msg = f'{frame} stale'
                elif age > self.warning_age:
                    level = DiagnosticStatus.WARN
                    msg = f'{frame} aging'
                else:
                    level = DiagnosticStatus.OK
                    msg = f'{frame} fresh'

                self._emit_status(status, level, 'tf_gate', msg, [KeyValue(key='age_s', value=f'{age:.4f}')])
            except Exception:
                self._emit_status(status, DiagnosticStatus.ERROR, 'tf_gate', f'no transform {base_frame}->{frame}')

    def _evaluate(self):
        status_items = []
        self._check_joint_limits(status_items)
        self._check_execution_error(status_items)
        self._check_planning_streak(status_items)
        self._check_tf(status_items)

        overall = max((s.level for s in status_items), default=DiagnosticStatus.OK)
        self.ok_pub.publish(Bool(data=overall != DiagnosticStatus.ERROR))

        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.status = status_items
        self.diag_pub.publish(msg)


def main(argv=None):
    rclpy.init(args=argv)
    node = SafetyWatchdogNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
