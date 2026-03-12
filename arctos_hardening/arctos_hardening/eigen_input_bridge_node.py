#!/usr/bin/env python3

from __future__ import annotations

import numpy as np

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from threading import Lock


class EigenInputBridgeNode(Node):
    def __init__(self):
        super().__init__('eigen_input_bridge_node')

        self.declare_parameter('source_topic', '/arctos/eigen/source_matrix')
        self.declare_parameter('output_topic', '/arctos/hardening/eigen/input_cov')
        self.declare_parameter('matrix_size', 6)
        self.declare_parameter('enable_fallback_from_eigendecomp', True)
        self.declare_parameter('values_topic', '/arctos/eigen/values')
        self.declare_parameter('vectors_topic', '/arctos/eigen/vectors')

        self.matrix_size = int(self.get_parameter('matrix_size').value)
        self.enable_fallback = bool(self.get_parameter('enable_fallback_from_eigendecomp').value)
        self.output_pub = self.create_publisher(
            Float64MultiArray,
            self.get_parameter('output_topic').value,
            10,
        )

        self.create_subscription(
            Float64MultiArray,
            self.get_parameter('source_topic').value,
            self._on_matrix,
            10,
        )
        self._latest_values = None
        self._latest_vectors = None
        self._lock = Lock()

        if self.enable_fallback:
            self.create_subscription(
                Float64MultiArray,
                self.get_parameter('values_topic').value,
                self._on_values,
                10,
            )
            self.create_subscription(
                Float64MultiArray,
                self.get_parameter('vectors_topic').value,
                self._on_vectors,
                10,
            )

            self.create_timer(0.25, self._publish_from_fallback)
            self._published_from_fallback = False

        self.get_logger().info(
            f'eigen input bridge active: {self.get_parameter("source_topic").value} -> '
            f'{self.get_parameter("output_topic").value} (N={self.matrix_size}), '
            f'fallback={self.enable_fallback}'
        )

    def _on_matrix(self, msg: Float64MultiArray):
        data = np.array(msg.data, dtype=float)
        expected = self.matrix_size * self.matrix_size
        if data.size != expected:
            self.get_logger().warn(
                f'eigen bridge ignored: expected {expected} values, got {data.size}'
            )
            return

        try:
            # accept both flat arrays and explicit dimensioned 2D arrays
            if msg.layout.dim:
                dim0 = msg.layout.dim[0].size
                dim1 = msg.layout.dim[1].size if len(msg.layout.dim) > 1 else self.matrix_size
                if dim0 != self.matrix_size or dim1 != self.matrix_size:
                    self.get_logger().warn(
                        f'eigen bridge ignored: layout {dim0}x{dim1}, expected {self.matrix_size}x{self.matrix_size}'
                    )
                    return
            data = data[:expected]
            mat = data.reshape((self.matrix_size, self.matrix_size))
            self._publish_matrix(mat)
        except Exception as exc:
            self.get_logger().error(f'eigen bridge failed: {exc}')

    def _on_values(self, msg: Float64MultiArray):
        with self._lock:
            self._latest_values = np.array(msg.data, dtype=float)

    def _on_vectors(self, msg: Float64MultiArray):
        with self._lock:
            self._latest_vectors = np.array(msg.data, dtype=float)

    def _publish_from_fallback(self):
        if not self.enable_fallback:
            return
        with self._lock:
            values = None if self._latest_values is None else self._latest_values.copy()
            vectors = None if self._latest_vectors is None else self._latest_vectors.copy()

        if values is None or vectors is None:
            return
        expected_v = self.matrix_size * self.matrix_size
        if values.size != self.matrix_size or vectors.size != expected_v:
            return

        try:
            v = vectors.reshape((self.matrix_size, self.matrix_size))
            if not np.all(np.isfinite(values)) or not np.all(np.isfinite(v)):
                return
            w = np.maximum(values, 0.0)
            mat = v @ np.diag(w) @ v.T
            self._publish_matrix(mat)
            if not self._published_from_fallback:
                self.get_logger().info('eigen bridge fallback active: reconstructed matrix from eigendecomp')
                self._published_from_fallback = True
        except Exception as exc:
            self.get_logger().warn(f'eigen fallback rebuild failed: {exc}')

    def _publish_matrix(self, mat: np.ndarray):
        out = Float64MultiArray()
        out.data = mat.reshape(-1).tolist()
        self.output_pub.publish(out)


def main(argv=None):
    rclpy.init(args=argv)
    node = EigenInputBridgeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
