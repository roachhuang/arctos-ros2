## Arctos Calibration + Perception Verification (Current Setup)

Use this checklist after updating `arctos_bringup/launch/camera_pose.launch.py`.

## Default Runtime Path (Minimal)

Primary pick/place flow should be:
`aruco_pose_node.py -> /detected_object_pose -> pose_stabilizer.py -> /detected_object_pose_stable -> mtc_node`

Use this as the default:

```bash
ros2 run arctos_bringup aruco_pose_node.py --ros-args \
  -p image_topic:=/kinect/image_raw \
  -p camera_info_topic:=/kinect/camera_info \
  -p output_topic:=/detected_object_pose \
  -p dictionary_id:=7 \
  -p marker_size:=0.02 \
  -p target_id:=5 \
  -p debug_log:=true

ros2 launch mtc_tutorial pick_place_demo.launch.py \
  use_detected_object_pose:=true \
  start_pose_stabilizer:=true \
  raw_detected_pose_topic:=/detected_object_pose \
  detected_pose_topic:=/detected_object_pose_stable \
  stable_repeat_count:=3 \
  stable_position_tolerance:=0.02 \
  stable_timeout_sec:=1.0 \
  detection_wait_timeout_sec:=10.0
```

## Optional Modules

These are optional and should not be part of the default runtime stack:
- `vision_guided_pick` (`use_vision_guided_pick:=true` in `my_moveit.launch.py`)
- `vision_tf_pose_bridge.py` (TF frame to pose topic bridge)
- `charuco_pose_node.py` (ChArUco board workflow)
- `point_and_inspect` tools

### 1. Launch one stack only

Hardware mode:
```bash
ros2 launch arctos_bringup my_moveit.launch.py \
  use_ros2_control:=true \
  use_fake_joint_states:=false \
  use_kinect:=true use_rviz:=true
```

Perception-only mode (no CAN/controller manager dependency):
```bash
ros2 launch arctos_bringup my_moveit.launch.py \
  use_ros2_control:=false \
  use_fake_joint_states:=true \
  use_kinect:=true use_rviz:=true
```

### 2. TF checks (must pass first)

```bash
ros2 run tf2_ros tf2_echo world base_link
ros2 run tf2_ros tf2_echo base_link kinect_rgb
ros2 run tf2_ros tf2_echo base_link kinect_depth
```

Pass:
- All transforms resolve after startup.
- `time 0.0` is normal for static TF.

### 3. Topic health checks

```bash
ros2 topic hz /kinect/points
ros2 topic info /moveit/filtered_cloud -v
ros2 topic hz /moveit/filtered_cloud
ros2 topic hz /joint_states
```

Pass:
- `/kinect/points` has stable nonzero rate.
- `/moveit/filtered_cloud` has one expected type and nonzero rate.
- `/joint_states` is present (real controllers or fake joint states).

### 4. Planning scene / octomap checks

Clear map once after startup:
```bash
ros2 service call /clear_octomap std_srvs/srv/Empty {}
```

In RViz:
- MotionPlanning -> Planning Scene -> Scene Geometry
- `Show Scene Geometry = true`
- `Voxel Rendering = Occupied Voxels`

Functional pass:
- Put an object in camera view and workspace.
- Occupied voxels appear in 3D planning scene.
- Plan through object: planner detours or fails.
- Remove object and replan: planning improves/succeeds.

### 5. Hand-eye validation (post-calibration acceptance)

1. Re-run MoveIt hand-eye with 10-15 new poses (hold-out set).
2. Compare new `base_link -> kinect_rgb` to saved transform.
3. Run task-space probe (robot TCP to known visual target).

Target acceptance:
- Repeatability drift: few mm, ~1-2 deg.
- Task-space error within your application tolerance (e.g. <= 10 mm for coarse pick/place).

### 6. Quick failure signatures

- `PlanningScene - requesting initial scene failed`:
  Check `/move_group` exists and planning-scene services are available.
- `Missing transform for shape mesh` in `move_group` logs:
  Robot link TF is incomplete; ensure `joint_states` are published.
- `/moveit/filtered_cloud` shows no rate:
  Point-cloud updater not active or TF/filtering is dropping points.

---
## Legacy Notes (Kept As-Is)

nmap -sP 192.168.1.0/24 | awk '/^Nmap/{ip=$NF}/B8:27:EB/{print ip}
# On Pi, improve WiFi stability
sudo nano /etc/dhcpcd.conf
# Add:
interface wlan0
metric 302  # Prefer WiFi over other interfaces

sudo nano /etc/wpa_supplicant/wpa_supplicant.conf
# Add:
ap_scan=1
country=US  # Your country code
# Stop unnecessary services during prints
sudo systemctl stop bluetooth
sudo systemctl stop avahi-daemon  # If not needed



colcon build --base-paths ~/ros2_ws/src/arctos
OR
colcon build --packages-select arctos_description --cmake-clean-cache

sudo apt install ros-jazz-rudf-tutorial
ros2 launch urdf_tutorial display.launch.py model:=/home/roach/ros2_ws/src/arctos/arctos_description/urdf/arctos.urdf.xacro

ros2 launch arctos_description simple_display.launch.py
ros2 run tf2_tools view_frames

* Enable canable usb:
    sudo ip link set can0 type can bitrate 500000
    sudo ip link set can0 up
    ip link show

    candump can0
    cansend can0 001#fd806402000c8070

    cansend can0 001#f502580200400092
    cansend can0 001#f5025802ffc00011
    stop motor in position mode4:
        cansend can0 001#01F5000004000000FA

    emergency stop:
        cansend can0 001#f7f8
    gohome till hit limit
        cansend can0 001#9192    
    read the RAW encoder value(addition):
        cansend can0 001#3536
    set current position to 0:
        cansend can0 001#9293
---------------------------------------------------------------------------
to show rclcpp_debug msg for h/w interface: ros2 logger set /arm_hardware_interface DEBUG
STEP 1:
    ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(xacro /home/roach/ros2_ws/src/arctos/arctos_description/urdf/arctos.urdf.xacro)"

STEP 2:
    # load h/w interface specified in the plugin of ros2_control xacro
    ros2 run controller_manager ros2_control_node --ros-args --params-file /home/roach/ros2_ws/src/arctos/arctos_bringup/config/real_controllers.yaml

STEP 3:
    ros2 run controller_manager spawner joint_state_broadcaster
    ros2 run controller_manager spawner arm_controller
    ros2 run controller_manager spawner gripper_controller

STEP 4:
    ros2 launch arctos_moveit_config move_group.launch.py
    ros2 run rviz2 rviz2 -d ~/ros2_ws/src/arctos/arctos_description/rviz/default.rviz
    in rviz2, add moveit_ros_visualization->MotionPlanning if not added yet.

ros2 node list
ros2 run rviz2 rviz2 -d /home/roach/ros2_ws/src/arctos/arctos_description/rviz/default.rviz

the above steps 1-3 is equivalent to ros2 launch arctos_bringup arctos.launch.py

step 1-4 is here:
    ros2 launch arctos_bringup my_moveit.launch.py |grep Arctos
    ros2 launch arctos_bringup my_moveit.launch.xml use_fake_hardware:=false 

add custom msg pkg - arctos_interfaces
    ros2 interface show arctos_interfaces/msg/PoseCommand
    add this line - "/home/roach/ros2_ws/install/arctos_interfaces/include/**" in "includePath" of .vscode->c_cpp_properties.json
--------------------------------------------------------------------------

test:
    (no need to add motion planning in rviz), links->tool_link->check Show Trail
    ros2 interface show example_interfaces/msg/Float64MultiArray
    
    ros2 run arctos_commander_cpp commander
    
    ros2 topic pub -1 /joint_cmd example_interfaces/msg/Float64MultiArray "{data: [1.0, 0.0, 0.0, 0.0, 0.0, 0.0]}"

    ros2 topic pub -1 /open_gripper example_interfaces/msg/Bool "{data: false}"

    ros2 topic pub -1 /pose_cmd arctos_interfaces/msg/PoseCommand "{x: -0.013, y: 0.001, z: 0.715, roll: 0.091, pitch: -0.391, yaw: -2.214, cartesian_path: false}"

    # Direct controller command (bypasses MoveIt):
    ros2 topic pub /arm_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "
    joint_names: ['X_joint', 'Y_joint', 'Z_joint', 'A_joint', 'B_joint', 'C_joint']
    points:
    - positions: [0.1, 0.0, 0.0, 0.0, 0.0, 0.0]
      time_from_start: {sec: 2, nanosec: 0}"



---------------------------------------------------------------------------
rqt_graph

ros2 control list_controllers
ros2 control list_controller_types
ros2 control list_hardware_interfaces
ros2 control list_hardware_components

ros2 topic echo /joint_states
ros2 topic list | grep arm_controller

ros2 launch arctos_bringup ros2_control.launch.py \
  --ros-args --log-level arctos_hardware_interface:=debug


applications:
    Safer motion with static obstacles (easiest)
    Use calibrated camera + MoveIt octomap to avoid table, walls, fixtures.

    Workspace monitoring / collision alerts
    Detect when new objects enter a keep-out zone and block motion.

    Point-and-inspect
    Click a point in camera view, transform to robot frame, move robot to inspect area.

    Vision-guided pick of known objects
    Detect object pose (e.g., ArUco/AprilTag/CAD match), transform to base_link, plan grasp.

    Bin picking with coarse perception
    Use depth clustering + grasp candidates for loosely arranged parts.

    Dynamic replanning around moving clutter
    Continuously update planning scene from depth cloud and replan in near real time.

    Precision assembly / insertion with visual servo assist (hardest)
    Use camera-based pose correction during approach to improve final alignment.

test:
    cd /home/roach/ros2_ws
    source /opt/ros/jazzy/setup.bash
    source install/local_setup.bash

    # pub detected_obj_pose. where target_id is the number the aruco marker represent.
    ros2 run arctos_bringup aruco_pose_node.py --ros-args   -p image_topic:=/kinect/image_raw   -p camera_info_topic:=/kinect/camera_info   -p output_topic:=/detected_object_pose   -p dictionary_id:=7   -p marker_size:=0.02   -p target_id:=-1   -p debug_log:=true

    # check pose 
    ros2 topic echo --once /detected_object_pose

    # sub detected obj pose
    ros2 launch arctos_bringup my_moveit.launch.py \
    use_vision_guided_pick:=true \
    vision_pick_execute:=true \
    vision_pick_object_pose_topic:=/detected_object_pose

    # manually pub for test
    ros2 topic pub --once /detected_object_pose geometry_msgs/msg/PoseStamped "{
        header: {frame_id: base_link},
        pose: {
            position: {x: 0.30, y: 0.00, z: 0.18},
            orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
        }
    }"


ros2 launch mtc_tutorial pick_place_demo.launch.py \
  use_detected_object_pose:=true \
  start_pose_stabilizer:=true \
  raw_detected_pose_topic:=/detected_object_pose \
  detected_pose_topic:=/detected_object_pose_stable \
  stable_repeat_count:=3 \
  stable_position_tolerance:=0.02 \
  stable_timeout_sec:=1.0 \
  detection_wait_timeout_sec:=10.0

pca (point cloud) for grasp:

ros2 launch arctos_moveit_config perception_stack.launch.py use_pca_grasp:=true pca_approach_axis:=smallest



