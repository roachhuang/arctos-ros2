#!/usr/bin/env python3

import math
from dataclasses import dataclass
from typing import Optional

import numpy as np

import rclpy
from geometry_msgs.msg import PoseStamped, Vector3Stamped, Point
from rclpy.node import Node
from rclpy.duration import Duration
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray


@dataclass(frozen=True)
class MarkerStyle:
    axis_length: float
    axis_width: float
    axis_head_width: float
    axis_head_length: float
    center_scale: float


def quaternion_to_rotation_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    """Convert quaternion to a 3x3 rotation matrix."""
    xx = x * x
    yy = y * y
    zz = z * z
    xy = x * y
    xz = x * z
    yz = y * z
    wx = w * x
    wy = w * y
    wz = w * z

    return np.array(
        [
            [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
            [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
            [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
        ],
        dtype=float,
    )


def rotation_matrix_to_quaternion(mat: np.ndarray) -> np.ndarray:
    """Convert a 3x3 rotation matrix to quaternion [x, y, z, w]."""
    m = mat
    t = m[0, 0] + m[1, 1] + m[2, 2]
    q = np.zeros(4, dtype=float)

    if t > 0.0:
        s = math.sqrt(t + 1.0) * 2.0
        q[3] = 0.25 * s
        q[0] = (m[2, 1] - m[1, 2]) / s
        q[1] = (m[0, 2] - m[2, 0]) / s
        q[2] = (m[1, 0] - m[0, 1]) / s
    elif (m[0, 0] > m[1, 1]) and (m[0, 0] > m[2, 2]):
        s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2.0
        q[3] = (m[2, 1] - m[1, 2]) / s
        q[0] = 0.25 * s
        q[1] = (m[0, 1] + m[1, 0]) / s
        q[2] = (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2.0
        q[3] = (m[0, 2] - m[2, 0]) / s
        q[0] = (m[0, 1] + m[1, 0]) / s
        q[1] = 0.25 * s
        q[2] = (m[1, 2] + m[2, 1]) / s
    else:
        s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2.0
        q[3] = (m[1, 0] - m[0, 1]) / s
        q[0] = (m[0, 2] + m[2, 0]) / s
        q[1] = (m[1, 2] + m[2, 1]) / s
        q[2] = 0.25 * s

    norm = np.linalg.norm(q)
    if norm < 1e-12:
        return np.array([0.0, 0.0, 0.0, 1.0], dtype=float)
    return q / norm


class PCAGraspNode(Node):
    def __init__(self) -> None:
        super().__init__("pca_grasp_node")

        self.pointcloud_topic = self.declare_parameter("pointcloud_topic", "/point_cloud").value
        self.target_frame = self.declare_parameter("target_frame", "base_link").value
        self.camera_optical_frame = self.declare_parameter(
            "camera_optical_frame", "camera_rgb_optical_frame"
        ).value

        self.min_points = int(self.declare_parameter("min_points", 120).value)
        self.approach_offset = float(self.declare_parameter("approach_offset", -0.12).value)
        self.grasp_offset = np.array(
            [
                float(self.declare_parameter("grasp_offset_x", 0.0).value),
                float(self.declare_parameter("grasp_offset_y", 0.0).value),
                float(self.declare_parameter("grasp_offset_z", 0.0).value),
            ],
            dtype=float,
        )

        self.marker_style = MarkerStyle(
            axis_length=float(self.declare_parameter("axis_length", 0.10).value),
            axis_width=float(self.declare_parameter("axis_width", 0.006).value),
            axis_head_width=float(self.declare_parameter("axis_head_width", 0.012).value),
            axis_head_length=float(self.declare_parameter("axis_head_length", 0.02).value),
            center_scale=float(self.declare_parameter("center_marker_scale", 0.01).value),
        )
        approach_axis_mode = self.declare_parameter("approach_axis", "largest").value

        self.grasp_pub = self.create_publisher(PoseStamped, "/pca_grasp/grasp_pose", 10)
        self.marker_pub = self.create_publisher(MarkerArray, "/pca_grasp/axis_markers", 10)
        self.sub = self.create_subscription(PointCloud2, self.pointcloud_topic, self.on_pointcloud, 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.prev_orientation: Optional[np.ndarray] = None
        self.busy = False

        self.approach_axis_mode = str(approach_axis_mode).strip().lower()
        if self.approach_axis_mode not in ("largest", "smallest"):
            self.get_logger().warn(
                f"Invalid approach_axis='{self.approach_axis_mode}', fallback to 'largest'."
            )
            self.approach_axis_mode = "largest"

        self.get_logger().info(
            f"pca_grasp_node ready (python). topic={self.pointcloud_topic} target_frame={self.target_frame} "
            f"min_points={self.min_points} approach_axis={self.approach_axis_mode}"
        )

    def on_pointcloud(self, msg: PointCloud2) -> None:
        if self.busy:
            self.get_logger().warn("Still processing previous cloud, ignoring new one.")
            return
        self.busy = True

        try:
            pts = self.extract_points(msg)
            pts = self.transform_cloud_to_target_frame(pts, msg)
            if pts is None:
                return

            if pts.shape[0] < self.min_points:
                self.get_logger().warn(
                    f"PCA input ignored: {pts.shape[0]} finite points (minimum {self.min_points} required)."
                )
                return

            try:
                center, frame = self.compute_pca_frame(pts)
            except RuntimeError as exc:
                self.get_logger().error(str(exc))
                return
            approach_axis_idx = 2 if self.approach_axis_mode == "largest" else 0
            self.enforce_axis_toward_camera(frame, approach_axis_idx)

            orientation = self.stabilize_orientation(rotation_matrix_to_quaternion(frame))
            pose_msg = self.build_grasp_pose(center, frame, approach_axis_idx, orientation, msg)
            self.grasp_pub.publish(pose_msg)

            self.publish_axis_markers(center, frame)
            self.get_logger().info("Published PCA grasp pose.")
        finally:
            self.busy = False

    def extract_points(self, msg: PointCloud2) -> np.ndarray:
        try:
            cloud_arr = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        except Exception as exc:
            self.get_logger().error(f"Failed to parse PointCloud2 points: {exc}")
            return np.empty((0, 3), dtype=float)

        if cloud_arr.size == 0:
            return np.empty((0, 3), dtype=float)
        return np.column_stack((cloud_arr["x"], cloud_arr["y"], cloud_arr["z"])).astype(float)

    def transform_cloud_to_target_frame(
        self, pts: np.ndarray, msg: PointCloud2
    ) -> Optional[np.ndarray]:
        if msg.header.frame_id == self.target_frame:
            return pts
        return self.transform_points(pts, msg.header.frame_id, self.target_frame, msg.header.stamp)

    def compute_pca_frame(self, pts: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        center = pts.mean(axis=0)
        centered_points = pts - center
        covariance = (centered_points.T @ centered_points) / float(pts.shape[0])

        try:
            _, eigenvectors = np.linalg.eigh(covariance)
        except Exception as exc:
            raise RuntimeError(f"PCA eigen decomposition failed: {exc}") from exc

        frame = np.column_stack((eigenvectors[:, 0], eigenvectors[:, 1], eigenvectors[:, 2]))
        if np.linalg.det(frame) < 0.0:
            frame[:, 2] *= -1.0
        return center, frame

    def stabilize_orientation(self, quaternion: np.ndarray) -> np.ndarray:
        if self.prev_orientation is not None and float(np.dot(quaternion, self.prev_orientation)) < 0.0:
            quaternion = -quaternion
        self.prev_orientation = quaternion.copy()
        return quaternion

    def build_grasp_pose(
        self,
        center: np.ndarray,
        frame: np.ndarray,
        approach_axis_idx: int,
        orientation: np.ndarray,
        msg: PointCloud2,
    ) -> PoseStamped:
        approach_axis = -frame[:, approach_axis_idx]
        grasp_pos = center + approach_axis * self.approach_offset + self.grasp_offset

        pose_msg = PoseStamped()
        pose_msg.header.frame_id = self.target_frame
        pose_msg.header.stamp = msg.header.stamp
        pose_msg.pose.position.x = float(grasp_pos[0])
        pose_msg.pose.position.y = float(grasp_pos[1])
        pose_msg.pose.position.z = float(grasp_pos[2])
        pose_msg.pose.orientation.x = float(orientation[0])
        pose_msg.pose.orientation.y = float(orientation[1])
        pose_msg.pose.orientation.z = float(orientation[2])
        pose_msg.pose.orientation.w = float(orientation[3])
        return pose_msg

    def transform_points(
        self, pts: np.ndarray, source_frame: str, target_frame: str, stamp
    ) -> Optional[np.ndarray]:
        try:
            tf_msg = self.tf_buffer.lookup_transform(
                target_frame, source_frame, stamp, timeout=Duration(seconds=0.2)
            )
        except (TransformException, Exception) as exc:
            self.get_logger().warn(
                f"Cannot transform cloud from '{source_frame}' to '{target_frame}': {exc}"
            )
            return None

        t = tf_msg.transform.translation
        q = tf_msg.transform.rotation
        rot = quaternion_to_rotation_matrix(q.x, q.y, q.z, q.w)
        trans = np.array([t.x, t.y, t.z], dtype=float)
        return (rot @ pts.T).T + trans

    def enforce_axis_toward_camera(self, frame: np.ndarray, approach_axis_idx: int) -> None:
        cam_z = Vector3Stamped()
        cam_z.header.frame_id = self.camera_optical_frame
        cam_z.header.stamp = self.get_clock().now().to_msg()
        cam_z.vector.x = 0.0
        cam_z.vector.y = 0.0
        cam_z.vector.z = 1.0

        try:
            cam_z_in_target = self.tf_buffer.transform(
                cam_z, self.target_frame, timeout=Duration(seconds=0.2)
            )
        except (TransformException, Exception) as exc:
            self.get_logger().warn(
                f"PCA safeguard: cannot transform camera optical z-axis from '{self.camera_optical_frame}' "
                f"to '{self.target_frame}': {exc}"
            )
            return

        cam_axis = np.array(
            [cam_z_in_target.vector.x, cam_z_in_target.vector.y, cam_z_in_target.vector.z],
            dtype=float,
        )
        c_norm = np.linalg.norm(cam_axis)
        if c_norm < 1e-8:
            return
        cam_axis /= c_norm

        approach_axis = frame[:, approach_axis_idx]
        if np.dot(approach_axis, cam_axis) < 0.0:
            # Keep the basis right-handed while aligning approach axis with camera_z:
            # flip approach axis and another basis axis => determinant sign unchanged.
            frame[:, approach_axis_idx] *= -1.0
            helper_idx = (approach_axis_idx + 1) % 3
            frame[:, helper_idx] *= -1.0

    def publish_axis_markers(self, center: np.ndarray, frame: np.ndarray) -> None:
        markers = MarkerArray()
        axes = [frame[:, 0], frame[:, 1], frame[:, 2]]
        colors = [
            (1.0, 0.0, 0.0),
            (0.0, 1.0, 0.0),
            (0.0, 0.0, 1.0),
        ]

        for i, axis in enumerate(axes):
            mk = Marker()
            mk.header.frame_id = self.target_frame
            mk.header.stamp = self.get_clock().now().to_msg()
            mk.ns = "pca_axes"
            mk.id = i
            mk.type = Marker.ARROW
            mk.action = Marker.ADD
            mk.lifetime = Duration(seconds=0.0).to_msg()
            mk.scale.x = self.marker_style.axis_width
            mk.scale.y = self.marker_style.axis_head_width
            mk.scale.z = self.marker_style.axis_head_length
            mk.color.a = 0.9
            mk.color.r, mk.color.g, mk.color.b = colors[i]

            p0 = Point(x=float(center[0]), y=float(center[1]), z=float(center[2]))
            p1 = Point(
                x=float(center[0] + axis[0] * self.marker_style.axis_length),
                y=float(center[1] + axis[1] * self.marker_style.axis_length),
                z=float(center[2] + axis[2] * self.marker_style.axis_length),
            )
            mk.points = [p0, p1]
            markers.markers.append(mk)

        center_marker = Marker()
        center_marker.header.frame_id = self.target_frame
        center_marker.header.stamp = self.get_clock().now().to_msg()
        center_marker.ns = "pca_axes"
        center_marker.id = 999
        center_marker.type = Marker.SPHERE
        center_marker.action = Marker.ADD
        center_marker.lifetime = Duration(seconds=0.0).to_msg()
        center_marker.pose.orientation.w = 1.0
        center_marker.pose.position = Point(x=float(center[0]), y=float(center[1]), z=float(center[2]))
        center_marker.scale.x = self.marker_style.center_scale
        center_marker.scale.y = self.marker_style.center_scale
        center_marker.scale.z = self.marker_style.center_scale
        center_marker.color.a = 0.8
        center_marker.color.r = 1.0
        center_marker.color.g = 1.0
        center_marker.color.b = 1.0

        markers.markers.append(center_marker)
        self.marker_pub.publish(markers)


def main() -> None:
    rclpy.init()
    node = PCAGraspNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
