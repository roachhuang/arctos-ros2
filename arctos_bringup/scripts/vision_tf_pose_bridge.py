#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
from rclpy.node import Node
from tf2_ros import Buffer, TransformException, TransformListener


class VisionTfPoseBridge(Node):
    def __init__(self) -> None:
        super().__init__("vision_tf_pose_bridge")

        self.source_frame = self.declare_parameter("source_frame", "kinect_rgb").value
        self.object_frame = self.declare_parameter("object_frame", "detected_object").value
        self.output_topic = self.declare_parameter("output_topic", "/detected_object_pose").value
        self.publish_rate_hz = float(self.declare_parameter("publish_rate_hz", 10.0).value)
        self.lookup_timeout_sec = float(self.declare_parameter("lookup_timeout_sec", 0.1).value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pose_pub = self.create_publisher(PoseStamped, self.output_topic, 10)

        period = 1.0 / max(self.publish_rate_hz, 0.1)
        self.timer = self.create_timer(period, self.on_timer)

    def on_timer(self) -> None:
        try:
            tf = self.tf_buffer.lookup_transform(
                self.source_frame,
                self.object_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=self.lookup_timeout_sec),
            )
        except TransformException:
            return

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.source_frame
        msg.pose.position.x = tf.transform.translation.x
        msg.pose.position.y = tf.transform.translation.y
        msg.pose.position.z = tf.transform.translation.z
        msg.pose.orientation = tf.transform.rotation
        self.pose_pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = VisionTfPoseBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
