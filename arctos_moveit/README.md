

. install/setup.bash
in urdf folder
ros2 run xacro xacro arctos.urdf.xacro > arctos.urdf
check_urdf arctos.urdf
cd ~/ros_ws
ros2 launch moveit_setup_assistant setup_assistant.launch.py

# use this cmd to see which .h or hpp needs to be included for the lib
dpkg -L ros-humble-moveit-ros-planning-interface | grep .h

To write MoveIt 2 Python code in your ros2_ws that uses the MoveIt 2 Python libraries you built from source in ws_moveit, you need to ensure that the Python environment in your ros2_ws has access to the libraries installed in ws_moveit. Here's the standard procedure:
    1. Build your ws_moveit workspace:

    You've already done this by building MoveIt 2 from source. Ensure that the build process completed successfully without errors. The Python libraries will typically be installed in the install/lib/python3.x/site-packages directory within ws_moveit.

    2. Source your ws_moveit workspace:

    Before running any Python code in your ros2_ws that intends to use the MoveIt 2 libraries, you need to source the setup file from your ws_moveit workspace in your terminal. This will add the necessary paths to your environment.

    cd ~/path/to/your/ws_moveit  # Navigate to the root of your ws_moveit workspace
    source install/setup.bash   # Or setup.zsh, depending on your shell

    cd ~/path/to/your/ros2_ws    # Navigate to the root of your ros2_ws workspace
    colcon build
    source install/setup.bash     # Or setup.zsh