#!/bin/bash
# Launch the real Arctos arm (hardware interface + controllers + MoveIt2).

cleanup() {
    echo "Cleaning up..."
    sleep 5.0
    sudo pkill -9 -f "ros2|rviz2|robot_state_publisher|joint_state_publisher|moveit|move_group"
}

# Set up cleanup trap
trap cleanup INT TERM

# Reset can0 to a known state (bitrate + auto-recovery on bus-off) before launch
if ip link show can0 >/dev/null 2>&1; then
    sudo ip link set can0 down
    sleep 1
    sudo ip link set can0 type can bitrate 500000 restart-ms 100
    sudo ip link set can0 up
fi

echo "Launching real robot..."
ros2 launch arctos_bringup my_moveit.launch.py \
    use_ros2_control:=true \
    use_fake_joint_states:=false \
    use_kinect:=false \
    use_rviz:=true
