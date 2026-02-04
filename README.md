# Arctos ROS2 Control Package

Official ROS2 package for controlling the Arctos robotic arm, featuring hardware integration over CAN and real-time motion control using ros2_control.

## Table of Contents

- [Prerequisites](#prerequisites)
- [Installation](#installation)
- [Configuration](#configuration)
- [Building](#building)
- [Running](#running)
- [Usage](#usage)
- [Troubleshooting](#troubleshooting)

## Prerequisites

### System Requirements

- ROS 2 Jazzy
- Ubuntu 24.04 (Noble)
- Python 3.10+

### USB Port Permissions

Enable communication with the hardware by adding your user to the `dialout` group:

```bash
sudo usermod -aG dialout $USER
sudo usermod -aG realtime $USER
# Log out and log back in for changes to take effect
```

### Required Packages

```bash
sudo apt-get update && sudo apt-get install -y \
    libboost-all-dev \
    ros-jazzy-hardware-interface \
    ros-jazzy-controller-manager \
    ros-jazzy-ros2-controllers \
    ros-jazzy-tf-transformations \
    ros-jazzy-gz* \
    ros-jazzy-pal-statistics \
    ros-jazzy-moveit-* \
    ros-jazzy-moveit-kinematics
```

## Installation

### 1. Environment Setup

Add the following to your `~/.bashrc`:

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export GZ_SIM_SYSTEM_PLUGIN_PATH=$GZ_SIM_SYSTEM_PLUGIN_PATH:/opt/ros/jazzy/lib

source /opt/ros/jazzy/setup.bash
source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
source ~/ros2_ws/install/setup.bash
```

### 2. Clone ArctosGUI (Optional)

```bash
git clone https://github.com/ArctosRobotics/ArctosGUI
cd arctosgui
pip3 install -r requirements.txt
./run.sh
```

## Configuration

### URDF Generation

Generate the URDF from Xacro:

```bash
cd ~/ros2_ws/src/arctos/urdf
ros2 run xacro xacro arctos.urdf.xacro > arctos.urdf
check_urdf arctos.urdf
```

### Verify URDF

```bash
ros2 launch urdf_tutorial display.launch.py model:=/home/roach/ros2_ws/src/arctos/urdf/arctos.urdf
```

## Building

### Build the Workspace

```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

### Build Specific Packages

```bash
colcon build --packages-select arctos --symlink-install
colcon build --packages-select arctos_alex --symlink-install
```

### Install Dependencies

```bash
rosdep install -y -r --rosdistro jazzy --from-paths src
```

## Running

### Start Hardware Interface

```bash
ros2 launch arctos_bringup real_robot.launch.py
```

### Start Simulation

```bash
sh -x src/arctos/bringup/scripts/gz.sh
```

### Start MoveIt2

```bash
ros2 launch arctos_bringup my_moveit.launch.py
```

### Run Pick & Place Demo

```bash
ros2 launch mtc_tutorial pick_place_demo.launch.py
```

## Usage

### Check Available Controllers

```bash
ros2 control list_controllers
```

Expected output:
```
joint_state_broadcaster[active]
arm_controller[active]
```

### Send Joint Trajectory Commands

For `JointTrajectoryController`:

```bash
ros2 topic pub /arm_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{
  joint_names: ['X_joint', 'Y_joint', 'Z_joint', 'A_joint', 'B_joint', 'C_joint'],
  points: [
    {
      positions: [0.0, 0.5, 1.0, 0.0, -0.5, 0.0],
      time_from_start: {sec: 2, nanosec: 0}
    }
  ]
}"
```

### Send Position Commands

For `JointGroupPositionController`:

```bash
ros2 topic pub /arm_controller/commands std_msgs/msg/Float64MultiArray "{
  data: [0.0, 0.5, 1.0, 0.0, -0.5, 0.0]
}"
```

### Monitor Joint States

```bash
ros2 topic echo /joint_states
```

## Troubleshooting

### USB Device Not Found

```bash
ls /dev/ttyUSB*
sudo cat /dev/ttyUSB0
```

### Rebuild with Clean Cache

```bash
colcon build --packages-select <package_name> --cmake-clean-cache
```

### Hardware Interface Logs

```bash
ros2 launch arctos_bringup real_robot.launch.py | grep 'RobotArmInterface'
```

### Check ROS Distribution

```bash
echo $ROS_DISTRO
```

## Important Notes

- **Arduino Timing**: Arduino process time must be faster than the hardware interface send interval
- **Hardware Interface**: Do not flood Arduino with commands from ROS2 hardware interface
- **Controller Type**: Use `JointTrajectoryController` for MoveIt2 compatibility (not `parallel_gripper_action_controller`)
- **Gripper Control**: Gripper is controlled via `JointTrajectoryController` for MTC compatibility

## Package Structure

```
arctos/
├── arctos_bringup/          # Launch files and configurations
├── arctos_description/      # URDF and mesh files
├── arctos_hardware_interface/ # Hardware interface implementation
├── mtc_tutorial/            # Motion Task Constructor examples
└── my_arm_rl/              # Reinforcement learning examples
```

## References

- [ROS2 Control Documentation](https://control.ros.org/)
- [MoveIt2 Documentation](https://moveit.ros.org/)
- [Motion Task Constructor](https://github.com/ros-planning/moveit_task_constructor)
