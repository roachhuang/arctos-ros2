cd ~/ros2_ws/src/arctos_description/urdf
source ~/ros2_ws/install/setup.bash
ros2 run xacro xacro arctos.urdf.xacro > generated_robot.urdf
check_urdf generated_robot.urdf

rosrun xacro xacro.py `rospack find arctos_description`/urdf/arctos.urdf.xacro -o /tmp/arctos.urdf

pre-requiste:
    sudo apt install ros-jazzy-gz-ros2-control
    ensure that you have a dependency declared for gz_ros2_control in your package.xml

Troubleshooiting:
How to fix mesh not displayed in rviz?
    robotModel->Description Topic-> /robot_description

after add gripper on ros2_controllers.yaml, tf tree becomes incorrect.    
The hardware interface only handles the 6 arm joints, not the gripper joints. The gripper joints need to be handled separately or added to the hardware interface. Add gripper joint handling
