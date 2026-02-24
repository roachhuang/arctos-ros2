#!/usr/bin/env python3

import math
from typing import Optional

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image


def rotation_matrix_to_quaternion(rot: np.ndarray) -> tuple[float, float, float, float]:
    trace = float(rot[0, 0] + rot[1, 1] + rot[2, 2])
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * s
        qx = (rot[2, 1] - rot[1, 2]) / s
        qy = (rot[0, 2] - rot[2, 0]) / s
        qz = (rot[1, 0] - rot[0, 1]) / s
    elif rot[0, 0] > rot[1, 1] and rot[0, 0] > rot[2, 2]:
        s = math.sqrt(1.0 + rot[0, 0] - rot[1, 1] - rot[2, 2]) * 2.0
        qw = (rot[2, 1] - rot[1, 2]) / s
        qx = 0.25 * s
        qy = (rot[0, 1] + rot[1, 0]) / s
        qz = (rot[0, 2] + rot[2, 0]) / s
    elif rot[1, 1] > rot[2, 2]:
        s = math.sqrt(1.0 + rot[1, 1] - rot[0, 0] - rot[2, 2]) * 2.0
        qw = (rot[0, 2] - rot[2, 0]) / s
        qx = (rot[0, 1] + rot[1, 0]) / s
        qy = 0.25 * s
        qz = (rot[1, 2] + rot[2, 1]) / s
    else:
        s = math.sqrt(1.0 + rot[2, 2] - rot[0, 0] - rot[1, 1]) * 2.0
        qw = (rot[1, 0] - rot[0, 1]) / s
        qx = (rot[0, 2] + rot[2, 0]) / s
        qy = (rot[1, 2] + rot[2, 1]) / s
        qz = 0.25 * s
    return qx, qy, qz, qw


class CharucoPoseNode(Node):
    def __init__(self) -> None:
        super().__init__("charuco_pose_node")
        self.bridge = CvBridge()
        self.last_warn_ns = 0

        self.image_topic = self.declare_parameter("image_topic", "/kinect/image_raw").value
        self.camera_info_topic = self.declare_parameter("camera_info_topic", "/kinect/camera_info").value
        self.output_topic = self.declare_parameter("output_topic", "/detected_object_pose").value

        self.squares_x = int(self.declare_parameter("squares_x", 5).value)
        self.squares_y = int(self.declare_parameter("squares_y", 7).value)
        self.square_length = float(self.declare_parameter("square_length", 0.035).value)
        self.marker_length = float(self.declare_parameter("marker_length", 0.022).value)
        self.dictionary_id = int(self.declare_parameter("dictionary_id", int(cv2.aruco.DICT_5X5_250)).value)
        self.min_charuco_corners = int(self.declare_parameter("min_charuco_corners", 6).value)
        self.output_frame = self.declare_parameter("output_frame", "").value

        self.camera_matrix: Optional[np.ndarray] = None
        self.dist_coeffs: Optional[np.ndarray] = None
        self.camera_frame_id = ""

        self.dictionary = cv2.aruco.getPredefinedDictionary(self.dictionary_id)
        if hasattr(cv2.aruco, "CharucoBoard") and callable(getattr(cv2.aruco, "CharucoBoard")):
            self.board = cv2.aruco.CharucoBoard(
                (self.squares_x, self.squares_y),
                self.square_length,
                self.marker_length,
                self.dictionary,
            )
        else:
            self.board = cv2.aruco.CharucoBoard_create(
                self.squares_x,
                self.squares_y,
                self.square_length,
                self.marker_length,
                self.dictionary,
            )

        self.pose_pub = self.create_publisher(PoseStamped, self.output_topic, 10)
        self.info_sub = self.create_subscription(CameraInfo, self.camera_info_topic, self.on_camera_info, 10)
        self.image_sub = self.create_subscription(Image, self.image_topic, self.on_image, 10)

        self.get_logger().info(
            f"charuco_pose_node ready image={self.image_topic} camera_info={self.camera_info_topic} "
            f"output={self.output_topic} board={self.squares_x}x{self.squares_y}"
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
            self.throttled_warn("Waiting for camera_info before pose estimation.")
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self.throttled_warn(f"cv_bridge conversion failed: {exc}")
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _rejected = cv2.aruco.detectMarkers(gray, self.dictionary)
        if ids is None or len(ids) == 0:
            return

        valid, charuco_corners, charuco_ids = cv2.aruco.interpolateCornersCharuco(
            markerCorners=corners,
            markerIds=ids,
            image=gray,
            board=self.board,
            cameraMatrix=self.camera_matrix,
            distCoeffs=self.dist_coeffs,
        )
        if valid is None or int(valid) < self.min_charuco_corners:
            return

        ok, rvec, tvec = cv2.aruco.estimatePoseCharucoBoard(
            charucoCorners=charuco_corners,
            charucoIds=charuco_ids,
            board=self.board,
            cameraMatrix=self.camera_matrix,
            distCoeffs=self.dist_coeffs,
            rvec=None,
            tvec=None,
        )
        if not ok:
            return

        rot_mat, _ = cv2.Rodrigues(rvec)
        qx, qy, qz, qw = rotation_matrix_to_quaternion(rot_mat)

        out = PoseStamped()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.output_frame if self.output_frame else (
            self.camera_frame_id if self.camera_frame_id else msg.header.frame_id
        )
        out.pose.position.x = float(tvec[0])
        out.pose.position.y = float(tvec[1])
        out.pose.position.z = float(tvec[2])
        out.pose.orientation.x = qx
        out.pose.orientation.y = qy
        out.pose.orientation.z = qz
        out.pose.orientation.w = qw
        self.pose_pub.publish(out)


def main() -> None:
    rclpy.init()
    node = CharucoPoseNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
