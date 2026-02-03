arctos_bringup/bringup.launch.py
│
├── description.launch.py
│   ├── robot_state_publisher
│   │   └── Publishes /robot_description + TF tree
│   ├── joint_state_publisher (optional)
│   └── RViz2 (optional)
│
├── ros2_control.launch.py
│   └── controller_manager (ros2_control_node)
│       ├── Loads ros2_control.yaml
│       │
│       ├── Hardware Interface Plugin
│       │   arctos_hardware_interface/ArctosHardwareInterface
│       │   │
│       │   ├── on_init()
│       │   │   └── Allocates joint state vectors
│       │   │
│       │   ├── on_configure()
│       │   │   ├── Reads can_interface, vel, accel
│       │   │   ├── Reads joint can_id + gear_ratio
│       │   │   ├── Builds motors_[] vector
│       │   │   └── can_driver_.setMotors(motors_)
│       │   │
│       │   ├── on_activate()
│       │   │   ├── can_driver_.connect(can0)
│       │   │   ├── can_driver_.enableMotorSync()
│       │   │   ├── Query initial encoder positions
│       │   │   └── Sync position_states_ with hardware
│       │   │
│       │   ├── read()
│       │   │   ├── motors_snapshot = can_driver_.getMotorsSnapshot()
│       │   │   ├── Convert raw_position → rad
│       │   │   ├── Compute velocity
│       │   │   └── Update motors_[i].position_rad / velocity_rad
│       │   │
│       │   └── write()
│       │       ├── Convert rad → encoder counts
│       │       ├── Mark motors_[i].target_active = true
│       │       └── can_driver_.runPositionAbs()
│       │
│       ├── joint_state_broadcaster
│       │   └── Publishes /joint_states from HW interface
│       │
│       ├── arm_controller (JointTrajectoryController)
│       │   └── Provides /arm_controller/follow_joint_trajectory
│       │
│       └── gripper_controller (JointTrajectoryController)
│           └── Provides /gripper_controller/follow_joint_trajectory
│
├── arctos_homing_node
│   ├── /go_home (Trigger)
│   │   └── Calls hardware_interface.homeAllSync()
│   │       ├── can_driver_.homeSync()
│   │       └── can_driver_.setZeroSync()
│   │
│   ├── /dump_motors (Trigger)
│   │   └── can_driver_.logMotorsState()
│   │
│   └── /wait_all_converged (Trigger)
│       └── can_driver_.allMotionsConverged()
│
├── auto_home_client (optional)
│   └── Calls /go_home once at startup
│
└── move_group.launch.py (MoveIt)
    ├── move_group
    │   ├── Loads robot_description
    │   ├── Loads kinematics.yaml
    │   ├── Loads moveit_controllers.yaml
    │   │   ├── arm_controller → FollowJointTrajectory
    │   │   └── gripper_controller → FollowJointTrajectory
    │   └── Talks to ros2_control controllers
    │
    └── RViz2 (MoveIt config)
        └── Displays TF + joint states + planning scene

2. Data Flow Diagram (Sensors → Control → Actuators)
Encoder Feedback (CAN)
        │
        ▼
MksServoDriver::processCanFrame()
        │
        ▼
ArctosHardwareInterface::read()
        │
        ▼
joint_state_broadcaster → /joint_states
        │
        ▼
MoveIt + RViz
        │
        ▼
Trajectory from MoveIt
        │
        ▼
arm_controller (JointTrajectoryController)
        │
        ▼
ArctosHardwareInterface::write()
        │
        ▼
MksServoDriver::runPositionAbs()
        │
        ▼
CAN Bus → Motors

3. Control Flow Diagram (Startup Sequence)
bringup.launch.py
│
├── description.launch.py
│   └── robot_state_publisher publishes URDF
│
├── ros2_control.launch.py
│   └── controller_manager loads hardware + controllers
│
├── arctos_homing_node
│   └── exposes /go_home
│
├── auto_home_client (optional)
│   └── calls /go_home
│
└── move_group.launch.py
    └── MoveIt starts AFTER robot is homed

4. Homing Flow Diagram (Sync Mode)
User or auto_home_client calls /go_home
        │
        ▼
arctos_homing_node
        │
        ▼
ArctosHardwareInterface::homeAllSync()
        │
        ├── can_driver_.homeSync()
        │   (blocking until servo replies)
        │
        ├── can_driver_.setZeroSync()
        │   (blocking until servo replies)
        │
        └── Update motors[].is_homing = false
        ▼
Return success

A simplified “data flow” diagram
MoveIt Trajectory
        │
        ▼
JointTrajectoryController
        │
        ▼
ArctosHardwareInterface::write()
        │
        ▼
MksServoDriver::runPositionAbs()
        │
        ▼
CAN Bus → Motors
        │
        ▼
Encoder Feedback
        │
        ▼
MksServoDriver::processCanFrame()
        │
        ▼
ArctosHardwareInterface::read()
        │
        ▼
joint_state_broadcaster → RViz + MoveIt

# Arctos Bringup

The **Arctos Bringup** package provides the complete launch pipeline for the
Arctos robotic arm, including:

- URDF + robot description
- ros2_control hardware interface
- CAN‑based servo driver
- Joint trajectory controllers
- MoveIt integration
- Homing and diagnostic services

This package is the entry point for bringing the robot online.

---

## ✅ Features

- **Deterministic startup sequence**  
  Ensures hardware is initialized before controllers or MoveIt start.

- **Explicit homing workflow**  
  `/go_home` performs synchronous homing using CAN‑level sync commands.

- **Real‑time safe control loop**  
  `read()` and `write()` remain non‑blocking for ros2_control.

- **Unified Motor model**  
  Hardware interface and CAN driver share a consistent OOP motor structure.

- **MoveIt‑ready controllers**  
  `arm_controller` and `gripper_controller` expose standard
  `FollowJointTrajectory` interfaces.

---

## ✅ Launch Files

### `description.launch.py`
Loads:
- URDF (via xacro)
- robot_state_publisher
- optional joint_state_publisher
- optional RViz

### `ros2_control.launch.py`
Starts:
- controller_manager
- hardware interface plugin
- joint_state_broadcaster
- arm_controller
- gripper_controller

### `bringup.launch.py`
Top‑level launch that starts:
- description
- ros2_control
- homing/debug node
- optional auto‑home client
- MoveIt

---

## ✅ Homing Services

Provided by `arctos_homing_node`:

| Service | Description |
|--------|-------------|
| `/go_home` | Homes all joints using synchronous CAN commands |
| `/dump_motors` | Prints full motor state for debugging |
| `/wait_all_converged` | Waits until all joints reach target |

---

## ✅ MoveIt Integration

MoveIt uses the following controllers:

- `arm_controller` → 6‑DOF arm  
- `gripper_controller` → Right/Left jaw joints  

Both expose `follow_joint_trajectory` action servers.

---

## ✅ Bringup Sequence

1. Load URDF  
2. Start ros2_control  
3. Activate controllers  
4. Home robot (`/go_home`)  
5. Start MoveIt  
6. Plan and execute trajectories

---

## ✅ Directory Structure

