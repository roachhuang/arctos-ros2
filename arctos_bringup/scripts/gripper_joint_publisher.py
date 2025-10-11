#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class GripperJointPublisher(Node):
    def __init__(self):
        super().__init__('gripper_joint_publisher')
        self.publisher = self.create_publisher(JointState, '/joint_states', 10)
        self.timer = self.create_timer(0.1, self.publish_gripper_states)
        
    def publish_gripper_states(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['Left_jaw_joint', 'Right_jaw_joint']
        msg.position = [0.0, 0.0]
        msg.velocity = [0.0, 0.0]
        self.publisher.publish(msg)

def main():
    rclpy.init()
    node = GripperJointPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()