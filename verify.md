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
    cansend can0 001#fd806402000c8070

---------------------------------------------------------------------------
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

step 1-4 is here: ros2 launch arctos_bringup my_moveit.launch.xml use_fake_hardware:=false 

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