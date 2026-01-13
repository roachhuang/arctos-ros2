# ros2_arctos
Official ROS2 package for controlling the Arctos robotic arm, featuring hardware integration over CAN and real-time motion control using ros2_control.

insert these lines on the end of ~/.bashrc
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp 

    # export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:/home/roach/ros2_ws/install>
    # export GZ_SIM_RESOURCE_PATH=$HOME/.gz/models
    export GZ_SIM_SYSTEM_PLUGIN_PATH=$GZ_SIM_SYSTEM_PLUGIN_PATH:/opt/ros/jazzy/lib
    source /opt/ros/jazzy/setup.bash
    source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
    source ~/ros2_ws/install/setup.bash

    git clone https://github.com/ArctosRobotics/ArctosGUI cd arctosgui pip3 install -r requirements.txt ./run.sh
