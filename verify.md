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

step 1-4 is here: ros2 launch arctos_bringup my_moveit.launch.py 
planning in code: ros2 run arctos_commander_cpp test_moveit (no need to add motion planning in rviz)

---------------------------------------------------------------------------
rqt_graph

ros2 control list_controllers
ros2 control list_controller_types
ros2 control list_hardware_interfaces
ros2 control list_hardware_components