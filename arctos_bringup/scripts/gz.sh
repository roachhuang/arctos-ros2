#!/bin/bash
# Single script to launch the myCobot with Gazebo and ROS 2 Controllers

# https://github.com/automaticaddison/mycobot_ros2/tree/main
# launch 3 files: gz.launch.py, display.launch.py, and load_ros2_controllers.launch.py 
cleanup() {
    echo "Cleaning up..."
    sleep 5.0
    pkill -9 -f "ros2|gazebo|gz|nav2|amcl|bt_navigator|nav_to_pose|rviz2|assisted_teleop|cmd_vel_relay|robot_state_publisher|joint_state_publisher|move_to_free|mqtt|autodock|cliff_detection|moveit|move_group|basic_navigator"
}

# Set up cleanup trap
trap 'cleanup' SIGINT SIGTERM

echo "Launching Gazebo simulation..."
# ros2 launch robotarm_controller gz.launch.py \
ros2 launch bringup sim_robot.launch.py \
    use_sim_time:=true \
    # load_controllers:=true \
    # world_file:=empty_world.world \
    # use_camera:=true \
    # use_rviz:=true \
    # use_robot_state_pub:=true \    
    # x:=0.0 \
    # y:=0.0 \
    # z:=0.03 \
    # roll:=0.0 \
    # pitch:=0.0 \
    # yaw:=0.0