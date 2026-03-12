#!/bin/bash

clear
# Define the default value for use_fake_hardware
FAKE_HW_ARG=${1:-false}

cd ~/ros2_ws
if [ -f ~/ros2_ws/install/setup.bash ]; then
  source ~/ros2_ws/install/setup.bash 2>/dev/null
fi
if [ -f ~/ws_moveit2/install/setup.bash ]; then
  source ~/ws_moveit2/install/setup.bash 2>/dev/null
fi

source ~/.bashrc
# candown (Replaced with the alias's full command)
if ip link show can0 >/dev/null 2>&1; then
  sudo ip link set can0 down
  sleep 1
  sudo ip link set can0 type can bitrate 500000 restart-ms 100
  sudo ip link set can0 up
fi

# Use the value of the FAKE_HW_ARG variable in the launch command
ros2 launch arctos_bringup my_moveit.launch.py use_fake_hardware:=${FAKE_HW_ARG}
