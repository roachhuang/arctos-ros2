#!/bin/bash

clear
# Define the default value for use_fake_hardware
FAKE_HW_ARG=${1:-false}

cd ~/ros2_ws
source ~/ws_moveit2/install/setup.bash
source ~/ros2_ws/install/setup.bash

source ~/.bashrc
# candown (Replaced with the alias's full command)
sudo ip link set can0 down
sleep 1
sudo ip link set can0 type can bitrate 500000 restart-ms 100
sudo ip link set can0 up

# Use the value of the FAKE_HW_ARG variable in the launch command
ros2 launch arctos_bringup my_moveit.launch.py use_fake_hardware:=${FAKE_HW_ARG}
