#!/usr/bin/env python3

import math

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node


class PoseStabilizer(Node):
    def __init__(self) -> None:
        super().__init__("pose_stabilizer")

        self.input_topic = self.declare_parameter("input_topic", "/detected_object_pose").value
        self.output_topic = self.declare_parameter("output_topic", "/detected_object_pose_stable").value
        self.stable_repeat_count = int(self.declare_parameter("stable_repeat_count", 3).value)
        self.stable_position_tolerance = float(
            self.declare_parameter("stable_position_tolerance", 0.02).value
        )
        self.stable_timeout_sec = float(self.declare_parameter("stable_timeout_sec", 1.0).value)

        self.last_warn_ns = 0
        self.stable_count = 0
        self.has_last = False
        self.last_x = 0.0
        self.last_y = 0.0
        self.last_z = 0.0
        self.last_stamp = self.get_clock().now()
        self.stable_locked = False

        self.sub = self.create_subscription(PoseStamped, self.input_topic, self.on_pose, 10)
        self.pub = self.create_publisher(PoseStamped, self.output_topic, 10)

        self.get_logger().info(
            "pose_stabilizer ready input=%s output=%s repeats=%d tolerance=%.3f timeout=%.2fs"
            % (
                self.input_topic,
                self.output_topic,
                self.stable_repeat_count,
                self.stable_position_tolerance,
                self.stable_timeout_sec,
            )
        )

    def on_pose(self, msg: PoseStamped) -> None:
        now = self.get_clock().now()
        if self.has_last and (now - self.last_stamp).nanoseconds > int(self.stable_timeout_sec * 1e9):
            self.stable_count = 0
            self.stable_locked = False

        if not self.has_last or self.stable_count == 0:
            self.last_x = msg.pose.position.x
            self.last_y = msg.pose.position.y
            self.last_z = msg.pose.position.z
            self.last_stamp = now
            self.has_last = True
            self.stable_count = 1
            return

        dx = msg.pose.position.x - self.last_x
        dy = msg.pose.position.y - self.last_y
        dz = msg.pose.position.z - self.last_z
        dist = math.sqrt((dx * dx) + (dy * dy) + (dz * dz))

        if dist <= self.stable_position_tolerance:
            self.stable_count += 1
            self.last_x = msg.pose.position.x
            self.last_y = msg.pose.position.y
            self.last_z = msg.pose.position.z
            self.last_stamp = now
        else:
            self.last_x = msg.pose.position.x
            self.last_y = msg.pose.position.y
            self.last_z = msg.pose.position.z
            self.last_stamp = now
            self.stable_count = 1
            self.stable_locked = False
            return

        if self.stable_count >= self.stable_repeat_count:
            if not self.stable_locked:
                self.get_logger().info(
                    "Stable pose locked x=%.3f y=%.3f z=%.3f"
                    % (self.last_x, self.last_y, self.last_z)
                )
                self.stable_locked = True
            self.pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = PoseStabilizer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
