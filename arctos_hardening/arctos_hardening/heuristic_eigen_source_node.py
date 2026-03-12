#!/usr/bin/env python3

from __future__ import annotations

from typing import Dict, Optional

import numpy as np

import rclpy
from control_msgs.msg import DynamicJointState
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


def _extract_latest(values):
    return values[-1] if values else None


class HeuristicEigenSourceAdapterNode(Node):
    """
    Temporary heuristic adapter:
    generates a 6x6 matrix from joint state + dynamic joint state + PD-gain
    hints. Intended for integration testing only.
    """

    def __init__(self):
        super().__init__('heuristic_eigen_source_adapter_node')

        self.declare_parameter('output_topic', '/arctos/eigen/source_matrix')
        self.declare_parameter('matrix_size', 6)
        self.declare_parameter('publish_hz', 50.0)
        self.declare_parameter('joint_names', ['A_joint', 'B_joint', 'C_joint', 'X_joint', 'Y_joint', 'Z_joint'])
        self.declare_parameter('pd_p_gains', [40.0, 40.0, 40.0, 20.0, 20.0, 20.0])
        self.declare_parameter('pd_d_gains', [2.0, 2.0, 2.0, 1.0, 1.0, 1.0])
        self.declare_parameter('joint_weight', [1.0, 1.0, 1.0, 1.0, 1.0, 1.0])
        self.declare_parameter('source_joint_states_topic', '/joint_states')
        self.declare_parameter('source_dynamic_joint_states_topic', '/dynamic_joint_states')
        self.declare_parameter('source_timeout_s', 0.5)
        self.declare_parameter('publish_when_missing', True)
        self.declare_parameter('fallback_identity_scale', 0.0)
        self.declare_parameter('min_joint_stiffness', 0.02)
        self.declare_parameter('max_joint_stiffness', 8.0)
        self.declare_parameter('gain_scale', 1.0e-3)
        self.declare_parameter('vel_scale', 0.15)
        self.declare_parameter('eff_scale', 0.01)
        self.declare_parameter('smooth_alpha', 0.25)

        self.output_topic = self.get_parameter('output_topic').value
        self.matrix_size = int(self.get_parameter('matrix_size').value)
        hz = float(self.get_parameter('publish_hz').value)
        hz = max(10.0, min(200.0, hz))
        period = 1.0 / hz

        self.joint_names = list(self.get_parameter('joint_names').value)
        self.joint_names = self.joint_names[: self.matrix_size]
        if len(self.joint_names) < self.matrix_size:
            for i in range(self.matrix_size - len(self.joint_names)):
                self.joint_names.append(f'heuristic_joint_{len(self.joint_names) + i}')
        self.p_gains = np.array(self.get_parameter('pd_p_gains').value, dtype=float)
        self.d_gains = np.array(self.get_parameter('pd_d_gains').value, dtype=float)
        self.joint_weight = np.array(self.get_parameter('joint_weight').value, dtype=float)

        self.source_joint_states_topic = self.get_parameter('source_joint_states_topic').value
        self.source_dynamic_joint_states_topic = self.get_parameter('source_dynamic_joint_states_topic').value
        self.source_timeout_s = float(self.get_parameter('source_timeout_s').value)
        self.publish_when_missing = bool(self.get_parameter('publish_when_missing').value)
        self.fallback_identity_scale = float(self.get_parameter('fallback_identity_scale').value)
        self.min_stiffness = float(self.get_parameter('min_joint_stiffness').value)
        self.max_stiffness = float(self.get_parameter('max_joint_stiffness').value)
        self.gain_scale = float(self.get_parameter('gain_scale').value)
        self.vel_scale = float(self.get_parameter('vel_scale').value)
        self.eff_scale = float(self.get_parameter('eff_scale').value)
        self.smooth_alpha = float(self.get_parameter('smooth_alpha').value)

        if self.p_gains.size < self.matrix_size:
            self.p_gains = np.pad(self.p_gains, (0, self.matrix_size - self.p_gains.size), constant_values=1.0)
        if self.d_gains.size < self.matrix_size:
            self.d_gains = np.pad(self.d_gains, (0, self.matrix_size - self.d_gains.size), constant_values=0.1)
        if self.joint_weight.size < self.matrix_size:
            self.joint_weight = np.pad(self.joint_weight, (0, self.matrix_size - self.joint_weight.size), constant_values=1.0)

        self.p_gains = self.p_gains[: self.matrix_size]
        self.d_gains = self.d_gains[: self.matrix_size]
        self.joint_weight = self.joint_weight[: self.matrix_size]

        self._joint_state: Dict[str, Dict[str, float]] = {}
        self._dynamic_state: Dict[str, Dict[str, float]] = {}
        self._joint_state_stamp_ns: Optional[int] = None
        self._dynamic_state_stamp_ns: Optional[int] = None
        self._last_diag = np.diag(np.full(self.matrix_size, self.fallback_identity_scale, dtype=float))

        self.publisher_ = self.create_publisher(Float64MultiArray, self.output_topic, 10)
        self.create_subscription(
            JointState,
            self.source_joint_states_topic,
            self._on_joint_state,
            10,
        )
        self.create_subscription(
            DynamicJointState,
            self.source_dynamic_joint_states_topic,
            self._on_dynamic_joint_state,
            10,
        )
        self.create_timer(period, self._on_timer)

        self.get_logger().info(
            f'heuristic eigen source active: '
            f'{self.source_joint_states_topic} + {self.source_dynamic_joint_states_topic} -> {self.output_topic} '
            f'({self.matrix_size}x{self.matrix_size}) at {hz:.1f} Hz'
        )
        self.get_logger().warn(
            'heuristic matrix source is NOT a true estimator covariance/eigen stream; use only as transitional adapter'
        )

    def _on_joint_state(self, msg: JointState):
        d = {}
        for i, name in enumerate(msg.name):
            if name not in self.joint_names:
                continue
            state = d.setdefault(name, {})
            if i < len(msg.position):
                state['position'] = float(msg.position[i])
            if i < len(msg.velocity):
                state['velocity'] = float(msg.velocity[i])
            if i < len(msg.effort):
                state['effort'] = float(msg.effort[i])
        self._joint_state = d
        self._joint_state_stamp_ns = self.get_clock().now().nanoseconds

    def _on_dynamic_joint_state(self, msg: DynamicJointState):
        d = {}
        for joint_name, iface_val in zip(msg.joint_names, msg.interface_values):
            if joint_name not in self.joint_names:
                continue
            state = d.setdefault(joint_name, {})
            for idx, raw_iface in enumerate(iface_val.interface_names):
                if idx >= len(iface_val.values):
                    continue
                iface = raw_iface.split('/')[-1].strip().lower()
                if iface.endswith('position'):
                    state['position'] = float(iface_val.values[idx])
                elif iface.endswith('velocity'):
                    state['velocity'] = float(iface_val.values[idx])
                elif iface in ('effort', 'force', 'torque'):
                    state['effort'] = float(iface_val.values[idx])
        if d:
            self._dynamic_state.update(d)
            self._dynamic_state_stamp_ns = self.get_clock().now().nanoseconds

    def _current_joint_value(self, name: str, key: str) -> Optional[float]:
        if name in self._dynamic_state and key in self._dynamic_state[name]:
            return self._dynamic_state[name][key]
        if name in self._joint_state and key in self._joint_state[name]:
            return self._joint_state[name][key]
        return None

    def _has_fresh_input(self, stamp_ns: Optional[int]) -> bool:
        if stamp_ns is None:
            return False
        age = (self.get_clock().now().nanoseconds - stamp_ns) * 1e-9
        return age <= self.source_timeout_s

    def _build_diag(self) -> Optional[np.ndarray]:
        use_joint = self._has_fresh_input(self._joint_state_stamp_ns) or self._has_fresh_input(self._dynamic_state_stamp_ns)
        if not use_joint and not self.publish_when_missing:
            return None

        diag = np.zeros(self.matrix_size, dtype=float)
        for i, name in enumerate(self.joint_names):
            p = float(self.p_gains[i])
            d = float(self.d_gains[i])
            vel = self._current_joint_value(name, 'velocity')
            eff = self._current_joint_value(name, 'effort')
            if vel is None:
                vel = 0.0
            if eff is None:
                eff = 0.0

            base = self.gain_scale * p
            dyn_term = self.vel_scale * abs(float(vel)) + self.eff_scale * abs(float(eff)) + self.gain_scale * d
            stiffness = base * (1.0 + dyn_term)
            stiffness = np.clip(stiffness * self.joint_weight[i], self.min_stiffness, self.max_stiffness)
            diag[i] = stiffness

        if self.fallback_identity_scale > 0.0 and use_joint and not np.any(np.isfinite(diag)):
            return np.full(self.matrix_size, float(self.fallback_identity_scale), dtype=float)
        if not use_joint and self.fallback_identity_scale > 0.0:
            return np.full(self.matrix_size, float(self.fallback_identity_scale), dtype=float)
        if not np.any(np.isfinite(diag)):
            return None

        return diag

    def _on_timer(self):
        diag = self._build_diag()
        if diag is None:
            return

        diag = np.maximum(diag, self.min_stiffness)
        mat = np.diag(diag)
        alpha = min(1.0, max(0.0, self.smooth_alpha))
        mat = (1.0 - alpha) * self._last_diag + alpha * mat
        self._last_diag = mat

        out = Float64MultiArray()
        out.data = mat.reshape(-1).tolist()
        self.publisher_.publish(out)


def main(argv=None):
    rclpy.init(args=argv)
    node = HeuristicEigenSourceAdapterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
