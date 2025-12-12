follow this link to install moveit2 for jazzy
https://moveit.picknik.ai/main/doc/tutorials/pick_and_place_with_moveit_task_constructor/pick_and_place_with_moveit_task_constructor.html#

cd ~/ws_moveit/src
git clone -b jazzy https://github.com/moveit/moveit_task_constructor.git

cd ~/ros2_ws
sudo apt update && rosdep install -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y --skip-keys "remote_interface warehouse_ros_mongo"

cd src/move_it_task_constructor.git
git checkout jazzy
cd ~/ros2_ws

coclon build --mixin debug
coclon build --mixin release

test it:
    sh -x a.sh
    ros2 launch moveit_task_constructor_demo run.launch.py exe:=pick_place_demo


//////////////////////////
sudo apt install ros-jazzy-moveit-task-constructor-core
sudo apt install ros-jazzy-moveit-task-constructor-visualization
sudo apt install ros-jazzy-moveit-task-constructor-capabilities
