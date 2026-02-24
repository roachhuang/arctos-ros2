#!/usr/bin/env python3

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image


def rvec_to_quaternion(rvec: np.ndarray) -> tuple[float, float, float, float]:
    rot_mat, _ = cv2.Rodrigues(rvec)
    trace = float(rot_mat[0, 0] + rot_mat[1, 1] + rot_mat[2, 2])
    if trace > 0.0:
        s = (trace + 1.0) ** 0.5 * 2.0
        qw = 0.25 * s
        qx = (rot_mat[2, 1] - rot_mat[1, 2]) / s
        qy = (rot_mat[0, 2] - rot_mat[2, 0]) / s
        qz = (rot_mat[1, 0] - rot_mat[0, 1]) / s
    elif rot_mat[0, 0] > rot_mat[1, 1] and rot_mat[0, 0] > rot_mat[2, 2]:
        s = (1.0 + rot_mat[0, 0] - rot_mat[1, 1] - rot_mat[2, 2]) ** 0.5 * 2.0
        qw = (rot_mat[2, 1] - rot_mat[1, 2]) / s
        qx = 0.25 * s
        qy = (rot_mat[0, 1] + rot_mat[1, 0]) / s
        qz = (rot_mat[0, 2] + rot_mat[2, 0]) / s
    elif rot_mat[1, 1] > rot_mat[2, 2]:
        s = (1.0 + rot_mat[1, 1] - rot_mat[0, 0] - rot_mat[2, 2]) ** 0.5 * 2.0
        qw = (rot_mat[0, 2] - rot_mat[2, 0]) / s
        qx = (rot_mat[0, 1] + rot_mat[1, 0]) / s
        qy = 0.25 * s
        qz = (rot_mat[1, 2] + rot_mat[2, 1]) / s
    else:
        s = (1.0 + rot_mat[2, 2] - rot_mat[0, 0] - rot_mat[1, 1]) ** 0.5 * 2.0
        qw = (rot_mat[1, 0] - rot_mat[0, 1]) / s
        qx = (rot_mat[0, 2] + rot_mat[2, 0]) / s
        qy = (rot_mat[1, 2] + rot_mat[2, 1]) / s
        qz = 0.25 * s
    return float(qx), float(qy), float(qz), float(qw)


class ArucoPoseNode(Node):
    def __init__(self) -> None:
        super().__init__("aruco_pose_node")
        self.bridge = CvBridge()
        self.last_warn_ns = 0

        self.image_topic = self.declare_parameter("image_topic", "/kinect/image_raw").value
        self.camera_info_topic = self.declare_parameter("camera_info_topic", "/kinect/camera_info").value
        self.output_topic = self.declare_parameter("output_topic", "/detected_object_pose").value
        self.marker_size = float(self.declare_parameter("marker_size", 0.02).value)
        self.dictionary_id = int(self.declare_parameter("dictionary_id", int(cv2.aruco.DICT_5X5_250)).value)
        self.target_id = int(self.declare_parameter("target_id", -1).value)
        self.debug_log = bool(self.declare_parameter("debug_log", False).value)

        self.camera_matrix = None
        self.dist_coeffs = None
        self.camera_frame_id = ""

        self.dictionary = cv2.aruco.getPredefinedDictionary(self.dictionary_id)
        if hasattr(cv2.aruco, "DetectorParameters_create"):
            self.detector_params = cv2.aruco.DetectorParameters_create()
        else:
            self.detector_params = cv2.aruco.DetectorParameters()

        half = self.marker_size / 2.0
        # Marker object points in marker frame (z=0), clockwise from top-left.
        self.marker_object_points = np.array(
            [
                [-half, +half, 0.0],
                [+half, +half, 0.0],
                [+half, -half, 0.0],
                [-half, -half, 0.0],
            ],
            dtype=np.float32,
        )

        self.pose_pub = self.create_publisher(PoseStamped, self.output_topic, 10)
        self.info_sub = self.create_subscription(CameraInfo, self.camera_info_topic, self.on_camera_info, 10)
        self.image_sub = self.create_subscription(Image, self.image_topic, self.on_image, 10)

        self.get_logger().info(
            f"aruco_pose_node ready image={self.image_topic} camera_info={self.camera_info_topic} "
            f"output={self.output_topic} marker_size={self.marker_size}m target_id={self.target_id}"
        )

    def throttled_warn(self, msg: str, period_sec: float = 2.0) -> None:
        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self.last_warn_ns > int(period_sec * 1e9):
            self.get_logger().warn(msg)
            self.last_warn_ns = now_ns

    def on_camera_info(self, msg: CameraInfo) -> None:
        self.camera_matrix = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        self.dist_coeffs = np.array(msg.d, dtype=np.float64)
        self.camera_frame_id = msg.header.frame_id

    def on_image(self, msg: Image) -> None:
        if self.camera_matrix is None or self.dist_coeffs is None:
            self.throttled_warn("Waiting for camera_info before ArUco pose estimation.")
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            # Fallback for mono encodings.
            try:
                mono = self.bridge.imgmsg_to_cv2(msg, desired_encoding="mono8")
                frame = cv2.cvtColor(mono, cv2.COLOR_GRAY2BGR)
            except Exception:
                self.throttled_warn(f"cv_bridge conversion failed: {exc}")
                return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        corners, ids, _ = cv2.aruco.detectMarkers(gray, self.dictionary, parameters=self.detector_params)
        if ids is None or len(ids) == 0:
            if self.debug_log:
                self.throttled_warn("No ArUco markers detected in current frame.", period_sec=1.0)
            return

        ids_flat = ids.flatten().tolist()
        candidate_indices = list(range(len(ids_flat)))
        if self.target_id >= 0:
            candidate_indices = [i for i, marker_id in enumerate(ids_flat) if marker_id == self.target_id]
            if len(candidate_indices) == 0:
                if self.debug_log:
                    self.throttled_warn(f"Target marker id {self.target_id} not found in frame.", period_sec=1.0)
                return

        solved = False
        used_id = -1
        rvec = None
        tvec = None
        for idx in candidate_indices:
            image_points = np.asarray(corners[idx], dtype=np.float32).reshape(4, 2)
            ok, rvec_try, tvec_try = cv2.solvePnP(
                self.marker_object_points,
                image_points,
                self.camera_matrix,
                self.dist_coeffs,
                flags=cv2.SOLVEPNP_IPPE_SQUARE,
            )
            if not ok:
                ok, rvec_try, tvec_try = cv2.solvePnP(
                    self.marker_object_points,
                    image_points,
                    self.camera_matrix,
                    self.dist_coeffs,
                    flags=cv2.SOLVEPNP_ITERATIVE,
                )
            if ok:
                rvec = rvec_try.reshape(3)
                tvec = tvec_try.reshape(3)
                used_id = int(ids_flat[idx])
                solved = True
                break

        if not solved:
            if self.debug_log:
                self.throttled_warn("Markers detected but pose solvePnP failed.", period_sec=1.0)
            return

        qx, qy, qz, qw = rvec_to_quaternion(rvec)

        out = PoseStamped()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.camera_frame_id if self.camera_frame_id else msg.header.frame_id
        out.pose.position.x = float(tvec[0])
        out.pose.position.y = float(tvec[1])
        out.pose.position.z = float(tvec[2])
        out.pose.orientation.x = qx
        out.pose.orientation.y = qy
        out.pose.orientation.z = qz
        out.pose.orientation.w = qw
        self.pose_pub.publish(out)
        if self.debug_log:
            self.get_logger().info(
                f"Published marker id={used_id} pose z={out.pose.position.z:.3f}m",
                throttle_duration_sec=1.0,
            )


def main() -> None:
    rclpy.init()
    node = ArucoPoseNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
