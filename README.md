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

sudo apt-get update sudo apt-get install -y ros-jazzy-moveit-ros-visualization

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

Run from `~/ros2_ws` (workspace root), not from inside `src/arctos`.

```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

Use the release mixin instead when building `arctos_moveit_config` (required for that package):

```bash
colcon build --mixin release
source install/setup.bash
```

### Build Specific Packages

```bash
colcon build --packages-select arctos_hardware_interface --symlink-install
colcon build --packages-select arctos_bringup --symlink-install
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

### CAN Interface Not Found

```bash
ip link show can0
candump can0
```

### Rebuild with Clean Cache

```bash
colcon build --packages-select <package_name> --cmake-clean-cache
```

### Hardware Interface Logs

```bash
ros2 launch arctos_bringup my_moveit.launch.py use_ros2_control:=true use_fake_joint_states:=false | grep 'ArctosInterface'
```

### Check ROS Distribution

```bash
echo $ROS_DISTRO
```

### MoveIt Environment Conflicts (`ws_moveit2` vs `ros2_ws` vs apt)

Two MoveIt trees can exist side by side: a source-built `~/ws_moveit2` (full MoveIt2 + MoveIt Task
Constructor, sourced before `~/ros2_ws` in `.bashrc`) and apt's `ros-jazzy-moveit-*` binaries under
`/opt/ros/jazzy`. Symptoms of drift between them:

- **Duplicate MoveIt Task Constructor build**: if `ros2_ws/src` also contains its own
  `moveit_task_constructor` clone, `AMENT_PREFIX_PATH` ordering picks one copy arbitrarily
  (`ros2 pkg prefix moveit_task_constructor_core` to check). Fix: keep only one build — remove the
  duplicate clone from `ros2_ws/src` and its `build/`/`install/` artifacts, then rebuild.
- **`libgeometric_shapes.so.X.Y.Z: cannot open shared object file`**: `ws_moveit2` was built against an
  older apt `ros-jazzy-geometric-shapes` and the package was later upgraded (bumping its SONAME).
  Fix: rebuild the affected `ws_moveit2` packages (`ldd <binary> | grep geometric_shapes` to find which
  ones need a matching soname):
  ```bash
  cd ~/ws_moveit2
  colcon build --packages-select moveit_core moveit_ros_move_group moveit_ros_visualization ... \
      --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
  ```
- **`ModuleNotFoundError: No module named 'catkin_pkg'` during colcon build**: CMake's
  `find_package(Python3)` picked up a non-system Python (e.g. a `uv`-managed interpreter under
  `~/.local/bin` that's ahead of `/usr/bin` in `PATH`) which lacks `catkin_pkg`. Fix: pin the interpreter
  explicitly with `--cmake-args -DPython3_EXECUTABLE=/usr/bin/python3` on the colcon build.
- **`Could not find a package configuration file provided by "tf2_eigen_kdl"`**: apt package missing.
  Fix: `sudo apt-get update && sudo apt-get install -y ros-jazzy-tf2-eigen-kdl`.

### CAN Bus Reset Before Real-Hardware Launch

`realbot.sh` resets `can0` (bitrate 500000, `restart-ms 100` for bus-off auto-recovery) before every
launch. Check interface health manually with:

```bash
ip -s -d link show can0   # look for state ERROR-ACTIVE, 0 bus-errors/bus-off
candump can0               # confirm servos answer status queries
```

### CAN ID Enable Failures (`cmd=0xF3 status=0xFF`)

`status=0xFF` from `arctos_hardware_interface` is a **timeout sentinel**, not a byte from the servo —
it means no response frame came back for the Enable command within the timeout. If `candump` shows
servos answering position/status queries (`0x31`) but staying silent specifically to `0xF3` (Enable),
the CAN bus itself is healthy; check the servo drivers' main motor power rail (separate from CAN/logic
power) and any alarm/fault state on the driver boards.

### Duplicate Hardware Launches

Only run one real-hardware bringup (`realbot.sh`, `a.sh`, or manual `ros2 launch ... use_fake_hardware:=false`)
at a time. Two `ros2_control_node` instances opening the same `can0` socket and commanding the same
servos concurrently causes conflicting commands and unpredictable behavior. Check for a stray instance
with:

```bash
pgrep -af "ros2 launch arctos_bringup"
```

## Important Notes

- **CAN Timing**: Servo-side processing must be faster than the hardware interface's send interval
- **Hardware Interface**: Do not flood the CAN bus with commands from the `ros2_control` `write()` loop
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
