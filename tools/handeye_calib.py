#!/usr/bin/env python3
import argparse
import math
import time
import yaml

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import tf2_ros


def parse_checkerboard(size_str):
    parts = size_str.lower().split("x")
    if len(parts) != 2:
        raise ValueError("checkerboard size must be like 8x6")
    return int(parts[0]), int(parts[1])


def load_intrinsics(yaml_path):
    with open(yaml_path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f)
    k = np.array(data["camera_matrix"]["data"], dtype=np.float64).reshape(3, 3)
    d = np.array(data["distortion_coefficients"]["data"], dtype=np.float64).reshape(-1, 1)
    return k, d, data.get("distortion_model", "plumb_bob")


def rotvec_to_mat(rvec):
    rmat, _ = cv2.Rodrigues(rvec)
    return rmat


def mat_to_rotvec(rmat):
    rvec, _ = cv2.Rodrigues(rmat)
    return rvec


def tf_to_rt(tf_msg):
    t = tf_msg.transform.translation
    q = tf_msg.transform.rotation
    tvec = np.array([[t.x], [t.y], [t.z]], dtype=np.float64)
    # quaternion to rotation matrix
    qx, qy, qz, qw = q.x, q.y, q.z, q.w
    rmat = np.array(
        [
            [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
            [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
            [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)],
        ],
        dtype=np.float64,
    )
    return rmat, tvec


class HandEyeCalibrator(Node):
    def __init__(self, args):
        super().__init__("handeye_calibrator")
        self.bridge = CvBridge()
        self.image_topic = args.image
        self.base_frame = args.base
        self.gripper_frame = args.gripper
        self.cols, self.rows = parse_checkerboard(args.checkerboard)
        self.square = args.square
        self.required_samples = args.samples
        self.min_trans = args.min_trans
        self.min_rot = math.radians(args.min_rot_deg)
        self.k, self.d, _ = load_intrinsics(args.intrinsics)
        self.visualize = args.visualize

        self.objp = np.zeros((self.rows * self.cols, 3), np.float64)
        self.objp[:, :2] = np.mgrid[0:self.cols, 0:self.rows].T.reshape(-1, 2)
        self.objp *= self.square

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.sub = self.create_subscription(Image, self.image_topic, self.image_cb, 10)

        self.samples_r_world2cam = []
        self.samples_t_world2cam = []
        self.samples_r_base2gripper = []
        self.samples_t_base2gripper = []

        self.last_tf = None
        self.last_sample_time = 0.0
        self.get_logger().info(
            f"Waiting for images on {self.image_topic}. "
            f"Checkerboard: {self.cols}x{self.rows}, square {self.square} m."
        )

    def should_take_sample(self, r_base2gripper, t_base2gripper):
        if self.last_tf is None:
            return True
        r_last, t_last = self.last_tf
        # translation delta
        dt = np.linalg.norm(t_base2gripper - t_last)
        # rotation delta
        r_delta = r_last.T @ r_base2gripper
        angle = math.acos(max(min((np.trace(r_delta) - 1.0) / 2.0, 1.0), -1.0))
        return dt >= self.min_trans or angle >= self.min_rot

    def image_cb(self, msg):
        try:
            tf_msg = self.tf_buffer.lookup_transform(
                self.gripper_frame, self.base_frame, rclpy.time.Time()
            )
        except Exception:
            return

        # tf is base -> gripper (target=gripper, source=base): ^gT_b
        r_base2gripper, t_base2gripper = tf_to_rt(tf_msg)
        if not self.should_take_sample(r_base2gripper, t_base2gripper):
            return

        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
        except Exception:
            return

        gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
        flags = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE
        ret, corners = cv2.findChessboardCorners(gray, (self.cols, self.rows), flags)
        if not ret:
            if self.visualize:
                cv2.imshow("handeye", img)
                cv2.waitKey(1)
            return

        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)

        ret, rvec, tvec = cv2.solvePnP(self.objp, corners2, self.k, self.d)
        if not ret:
            return

        if self.visualize:
            cv2.drawChessboardCorners(img, (self.cols, self.rows), corners2, ret)
            cv2.imshow("handeye", img)
            cv2.waitKey(1)

        self.samples_r_world2cam.append(rotvec_to_mat(rvec))
        self.samples_t_world2cam.append(tvec)
        self.samples_r_base2gripper.append(r_base2gripper)
        self.samples_t_base2gripper.append(t_base2gripper)
        self.last_tf = (r_base2gripper, t_base2gripper)
        self.last_sample_time = time.time()

        self.get_logger().info(
            f"Sample {len(self.samples_r_world2cam)}/{self.required_samples} captured"
        )

        if len(self.samples_r_world2cam) >= self.required_samples:
            self.compute()
            rclpy.shutdown()

    def compute(self):
        r_b2w, t_b2w, r_g2c, t_g2c = cv2.calibrateRobotWorldHandEye(
            self.samples_r_world2cam,
            self.samples_t_world2cam,
            self.samples_r_base2gripper,
            self.samples_t_base2gripper,
            method=cv2.CALIB_ROBOT_WORLD_HAND_EYE_SHAH,
        )

        # Compute base->camera from all samples and average.
        quats = []
        t_accum = []
        for r_g2b, t_g2b in zip(self.samples_r_base2gripper, self.samples_t_base2gripper):
            # ^cT_b = ^cT_g * ^gT_b
            r_c2b = r_g2c @ r_g2b
            t_c2b = r_g2c @ t_g2b + t_g2c
            # invert to get ^bT_c
            r_b2c = r_c2b.T
            t_b2c = -r_c2b.T @ t_c2b
            t_accum.append(t_b2c.reshape(3))
            # rotation matrix to quaternion
            qw = math.sqrt(max(0.0, 1.0 + r_b2c[0, 0] + r_b2c[1, 1] + r_b2c[2, 2])) / 2.0
            qx = (r_b2c[2, 1] - r_b2c[1, 2]) / (4.0 * qw)
            qy = (r_b2c[0, 2] - r_b2c[2, 0]) / (4.0 * qw)
            qz = (r_b2c[1, 0] - r_b2c[0, 1]) / (4.0 * qw)
            quats.append(np.array([qx, qy, qz, qw], dtype=np.float64))

        t_mean = np.mean(np.stack(t_accum, axis=0), axis=0)
        q_mean = np.mean(np.stack(quats, axis=0), axis=0)
        q_mean = q_mean / np.linalg.norm(q_mean)
        qx, qy, qz, qw = q_mean.tolist()

        self.get_logger().info("--- Hand-eye result ---")
        self.get_logger().info(f"R_gripper2cam:\n{r_g2c}")
        self.get_logger().info(f"t_gripper2cam: {t_g2c.ravel().tolist()}")
        self.get_logger().info("--- Base -> Camera (mean over samples) ---")
        self.get_logger().info(f"t_base2cam: {t_mean.tolist()}")
        self.get_logger().info(
            "Static TF args (base_link -> camera): "
            f"{t_mean[0]:.4f} {t_mean[1]:.4f} {t_mean[2]:.4f} "
            f"{qx:.6f} {qy:.6f} {qz:.6f} {qw:.6f}"
        )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--image", default="/kinect/image_raw")
    parser.add_argument("--base", default="base_link")
    parser.add_argument("--gripper", default="Gripper_1")
    parser.add_argument("--checkerboard", required=True)
    parser.add_argument("--square", type=float, required=True)
    parser.add_argument("--intrinsics", required=True)
    parser.add_argument("--samples", type=int, default=15)
    parser.add_argument("--min-trans", type=float, default=0.02)
    parser.add_argument("--min-rot-deg", type=float, default=5.0)
    parser.add_argument("--visualize", action="store_true")
    args = parser.parse_args()

    rclpy.init()
    node = HandEyeCalibrator(args)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
