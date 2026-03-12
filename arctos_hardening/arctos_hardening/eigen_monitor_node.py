#!/usr/bin/env python3

import numpy as np

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Bool
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue


class EigenMonitorNode(Node):
    def __init__(self):
        super().__init__('eigen_monitor_node')

        self.declare_parameter('source_topic', '/arctos/hardening/eigen/input_cov')
        self.declare_parameter('matrix_size', 6)
        self.declare_parameter('eps_jitter', 1.0e-6)
        self.declare_parameter('min_eig_warning', 1.0e-6)
        self.declare_parameter('max_condition_number', 1.0e4)
        self.declare_parameter('values_topic', '/arctos/eigen/values')
        self.declare_parameter('vectors_topic', '/arctos/eigen/vectors')
        self.declare_parameter('diagnostic_topic', '/arctos/hardening/eigen')
        self.declare_parameter('ok_topic', '/arctos/hardening/eigen_ok')

        self.matrix_size = int(self.get_parameter('matrix_size').value)
        self.eps = float(self.get_parameter('eps_jitter').value)
        self.min_eig_warning = float(self.get_parameter('min_eig_warning').value)
        self.max_condition = float(self.get_parameter('max_condition_number').value)

        self.values_pub = self.create_publisher(Float64MultiArray, self.get_parameter('values_topic').value, 10)
        self.vectors_pub = self.create_publisher(Float64MultiArray, self.get_parameter('vectors_topic').value, 10)
        self.diag_pub = self.create_publisher(DiagnosticArray, self.get_parameter('diagnostic_topic').value, 10)
        self.ok_pub = self.create_publisher(Bool, self.get_parameter('ok_topic').value, 10)

        self.create_subscription(
            Float64MultiArray,
            self.get_parameter('source_topic').value,
            self._on_matrix,
            10,
        )

    def _publish_diag(self, level, name, message, extras=None):
        if extras is None:
            extras = {}
        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        status = DiagnosticStatus(level=level, name=name, message=message)
        for key, value in extras.items():
            status.values.append(KeyValue(key=str(key), value=str(value)))
        msg.status.append(status)
        self.diag_pub.publish(msg)

    def _on_matrix(self, msg: Float64MultiArray):
        expected = self.matrix_size * self.matrix_size
        if len(msg.data) != expected:
            self._publish_diag(DiagnosticStatus.ERROR, 'eigen_input',
                               f'expected {expected} values, got {len(msg.data)}')
            self.ok_pub.publish(Bool(data=False))
            return

        A = np.array(msg.data, dtype=float).reshape((self.matrix_size, self.matrix_size))
        if not np.isfinite(A).all():
            self._publish_diag(DiagnosticStatus.ERROR, 'eigen_input', 'non-finite matrix values')
            self.ok_pub.publish(Bool(data=False))
            return

        A = 0.5 * (A + A.T)
        A = A + self.eps * np.eye(self.matrix_size)

        try:
            w, v = np.linalg.eigh(A)
        except Exception as exc:
            self._publish_diag(DiagnosticStatus.ERROR, 'lin_alg', str(exc))
            self.ok_pub.publish(Bool(data=False))
            return

        idx = np.argsort(w)[::-1]
        w = np.maximum(w[idx], 0.0)
        v = v[:, idx]
        total = float(np.sum(w))
        ratio = (w / total) if total > 0.0 else np.zeros_like(w)

        nonzero = w[w > self.min_eig_warning]
        cond = float(np.inf) if nonzero.size == 0 else float(nonzero.max() / max(nonzero.min(), self.min_eig_warning))

        status = DiagnosticStatus.OK
        reason = 'ok'
        if cond > self.max_condition:
            status = DiagnosticStatus.WARN
            reason = f'condition number high: {cond:.1e}'

        if ratio[0] > 0.98:
            status = DiagnosticStatus.WARN
            reason = f'mode collapse risk: ratio0={ratio[0]:.3f}'

        vals = Float64MultiArray()
        vals.data = w.tolist()
        self.values_pub.publish(vals)

        vecs = Float64MultiArray()
        vecs.data = v.reshape(-1).tolist()
        self.vectors_pub.publish(vecs)

        self._publish_diag(
            status,
            'eigen_output',
            reason,
            extras={
                'condition': cond,
                'ratio0': float(ratio[0]),
                'sum_top3': float(np.sum(w[:3])),
            },
        )
        self.ok_pub.publish(Bool(data=(status != DiagnosticStatus.ERROR)))


def main(argv=None):
    rclpy.init(args=argv)
    node = EigenMonitorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
