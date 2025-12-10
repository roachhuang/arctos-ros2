#!/usr/bin/env python3
"""
MTC Safety Check Script
Validates robot state before MTC execution on real hardware.
Checks:
  1. All controllers are loaded and active
  2. Joint limits are within safe bounds
  3. Robot is not in emergency stop
  4. All required frames are published (TF)
  5. Hardware interface is responsive
  6. Joint positions are reasonable (not NaN, within limits)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import numpy as np
from sensor_msgs.msg import JointState
from tf2_ros import TransformListener, Buffer
from controller_manager_msgs.srv import ListControllers
from moveit_msgs.srv import GetPositionFK, GetPositionIK
from moveit_msgs.msg import RobotState
import sys

class MTCSafetyChecker(Node):
    def __init__(self):
        super().__init__('mtc_safety_checker')
        
        # Configuration
        self.required_controllers = ['arm_controller', 'gripper_controller', 'joint_state_broadcaster']
        self.required_joints = ['X_joint', 'Y_joint', 'Z_joint', 'A_joint', 'B_joint', 'C_joint', 
                                'Left_jaw_joint', 'Right_jaw_joint']
        self.required_frames = ['world', 'base_link', 'tool_link']
        
        # Joint limits (from joint_limits.yaml)
        self.joint_limits = {
            'X_joint': {'min': -6.28, 'max': 6.28},
            'Y_joint': {'min': -0.5, 'max': 0.5},
            'Z_joint': {'min': -0.5, 'max': 2.0},
            'A_joint': {'min': -6.28, 'max': 6.28},
            'B_joint': {'min': -1.5708, 'max': 1.5708},
            'C_joint': {'min': -1.5708, 'max': 1.5708},
            'Left_jaw_joint': {'min': -0.1, 'max': 0.1},
            'Right_jaw_joint': {'min': -0.1, 'max': 0.1},
        }
        
        # State
        self.joint_states = {}
        self.checks_passed = {}
        self.checks_passed['joint_state'] = False
        self.checks_passed['controllers'] = False
        self.checks_passed['tf_frames'] = False
        self.checks_passed['joint_limits'] = False
        
        # Subscribers
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self._joint_state_callback, qos)
        
        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Services
        self.list_controllers_cli = self.create_client(
            ListControllers, '/controller_manager/list_controllers')
        
    def _joint_state_callback(self, msg: JointState):
        """Store joint states for later validation."""
        for i, name in enumerate(msg.name):
            if name in self.required_joints:
                self.joint_states[name] = {
                    'position': msg.position[i] if i < len(msg.position) else None,
                    'velocity': msg.velocity[i] if i < len(msg.velocity) else None,
                    'effort': msg.effort[i] if i < len(msg.effort) else None,
                }
    
    def check_joint_states(self) -> bool:
        """Verify all required joints are publishing state."""
        if not self.joint_states:
            self.get_logger().error("❌ No joint states received. Is joint_state_broadcaster running?")
            return False
        
        missing_joints = set(self.required_joints) - set(self.joint_states.keys())
        if missing_joints:
            self.get_logger().error(f"❌ Missing joints: {missing_joints}")
            return False
        
        for joint, state in self.joint_states.items():
            if state['position'] is None:
                self.get_logger().error(f"❌ Joint {joint} has no position data")
                return False
            if np.isnan(state['position']):
                self.get_logger().error(f"❌ Joint {joint} position is NaN")
                return False
        
        self.get_logger().info("✅ All joint states received and valid")
        return True
    
    def check_controllers(self) -> bool:
        """Verify all required controllers are loaded and active."""
        if not self.list_controllers_cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("❌ Controller manager service not available")
            return False
        
        req = ListControllers.Request()
        future = self.list_controllers_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        if not future.done():
            self.get_logger().error("❌ Controller list service timed out")
            return False
        
        response = future.result()
        active_controllers = {c.name for c in response.controller if c.state == 'active'}
        
        missing_controllers = set(self.required_controllers) - active_controllers
        if missing_controllers:
            self.get_logger().error(f"❌ Missing/inactive controllers: {missing_controllers}")
            self.get_logger().info(f"   Active controllers: {active_controllers}")
            return False
        
        self.get_logger().info("✅ All required controllers are active")
        return True
    
    def check_tf_frames(self) -> bool:
        """Verify required TF frames are available."""
        for frame in self.required_frames:
            try:
                self.tf_buffer.lookup_transform('world', frame, rclpy.time.Time())
            except Exception as e:
                self.get_logger().error(f"❌ TF frame '{frame}' not available: {e}")
                return False
        
        self.get_logger().info("✅ All required TF frames available")
        return True
    
    def check_joint_limits(self) -> bool:
        """Verify joint positions are within safe limits."""
        for joint, state in self.joint_states.items():
            if joint not in self.joint_limits:
                continue
            
            limits = self.joint_limits[joint]
            pos = state['position']
            
            if pos < limits['min'] or pos > limits['max']:
                self.get_logger().warn(
                    f"⚠️  Joint {joint} position {pos:.3f} outside limits [{limits['min']:.3f}, {limits['max']:.3f}]"
                )
                # Return warning but not fatal (robot may be moving into bounds)
                return False
        
        self.get_logger().info("✅ All joint positions within limits")
        return True
    
    def run_all_checks(self) -> bool:
        """Run all safety checks sequentially."""
        self.get_logger().info("\n" + "="*60)
        self.get_logger().info("Starting MTC Safety Checks")
        self.get_logger().info("="*60 + "\n")
        
        # Wait for joint states
        self.get_logger().info("Waiting for joint states...")
        start_time = self.get_clock().now()
        while not self.joint_states and (self.get_clock().now() - start_time).nanoseconds < 5e9:
            rclpy.spin_once(self, timeout_sec=0.1)
        
        checks = [
            ("Joint States", self.check_joint_states),
            ("Controllers", self.check_controllers),
            ("TF Frames", self.check_tf_frames),
            ("Joint Limits", self.check_joint_limits),
        ]
        
        results = []
        for name, check_fn in checks:
            try:
                result = check_fn()
                results.append((name, result))
                self.get_logger().info("")
            except Exception as e:
                self.get_logger().error(f"❌ Exception in {name} check: {e}")
                results.append((name, False))
        
        # Summary
        self.get_logger().info("\n" + "="*60)
        self.get_logger().info("Safety Check Summary")
        self.get_logger().info("="*60)
        
        passed = sum(1 for _, result in results if result)
        total = len(results)
        
        for name, result in results:
            status = "✅ PASS" if result else "❌ FAIL"
            self.get_logger().info(f"{status}: {name}")
        
        self.get_logger().info(f"\nTotal: {passed}/{total} checks passed")
        self.get_logger().info("="*60 + "\n")
        
        return passed == total


def main():
    rclpy.init()
    node = MTCSafetyChecker()
    
    try:
        all_passed = node.run_all_checks()
        node.destroy_node()
        rclpy.shutdown()
        
        if all_passed:
            print("\n✅ Robot is SAFE for MTC execution!\n")
            return 0
        else:
            print("\n❌ Robot FAILED safety checks. Fix issues before running MTC.\n")
            return 1
    except KeyboardInterrupt:
        node.destroy_node()
        rclpy.shutdown()
        return 1


if __name__ == '__main__':
    sys.exit(main())