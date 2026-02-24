sudo apt-get update
sudo apt-get install git cmake build-essential libusb-1.0-0-dev

sudo apt-get update
sudo apt-get install ros-jazzy-cv-bridge

diff --git a//home/roach/ros2_ws/src/kinect_ros2/include/kinect_ros2/kinect_ros2_component.hpp b//home/roach/ros2_ws/src/kinect_ros2/include/kinect_ros2/kinect_ros2_component.hpp
--- a//home/roach/ros2_ws/src/kinect_ros2/include/kinect_ros2/kinect_ros2_component.hpp
+++ b//home/roach/ros2_ws/src/kinect_ros2/include/kinect_ros2/kinect_ros2_component.hpp
@@ -11,3 +11,3 @@
 #include "sensor_msgs/msg/camera_info.hpp"
-#include "cv_bridge/cv_bridge.h"
+#include "cv_bridge/cv_bridge.hpp"

sudo apt-get update
sudo apt-get install libfreenect-dev freenect

cd ~
git clone https://github.com/OpenKinect/libfreenect
cd libfreenect
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
sudo make install
sudo ldconfig

mkdir -p ~/ws/src
cd ~/ws/src
git clone https://github.com/fadlio/kinect_ros2
cd ~/ws
rosdep install --from-paths src --ignore-src -r -y


source /opt/ros/jazzy/setup.bash
cd ~/ros2_ws
colcon build --base-paths src/kinect_ros2 --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release


 
uild fixes

Updated kinect_ros2 to include cv_bridge.hpp (Jazzy uses .hpp, not .h):
kinect_ros2_component.hpp
MoveIt perception config

Fixed malformed sensors_3d.yaml and configured the Kinect point cloud:
sensors_3d.yaml
Sensor key kinect_pointcloud → /kinect/points
MoveIt launch updates

Ensured the URDF is loaded into MoveIt:
arctos.urdf.xacro")
Added robot_state_publisher so base_link is published.
Added a static TF publisher (test values) from base_link → kinect_depth, with launch args for easy tuning.
Made RViz optional to reduce duplicate node spam.
move_group.launch.py
Results

Kinect point cloud publishes on /kinect/points.
MoveIt subscribes and produces /moveit/filtered_cloud.
Octomap shows “bricks” in RViz, so perception is working.
/monitored_planning_scene publishes (which is enough for MTC); /planning_scene staying quiet is normal and not a blocker.
If you want, I can also add a one‑shot launch that brings up Kinect + MoveIt + static TF together.


camera calibration:
    checkerboard 7x9, square size: 1"
    sudo apt-get update
    sudo apt-get install -y ros-jazzy-image-pipeline ros-jazzy-camera-calibration-parsers
   ros2 run camera_calibration cameracalibrator \
  --pattern chessboard \
  --size 8x6 --square 0.0254 \
  --no-service-check \
  --disable_calib_cb_fast_check \
  --ros-args \
  --remap image:=/kinect/image_raw \
  --remap camera:=/kinect
    move the board around till calibrate btn is active, click it and then save.
    ost.yaml will be created.

    extrinsic calibration:
        source /opt/ros/jazzy/setup.bash
        source /home/roach/ros2_ws/install/local_setup.bash

        ros2 launch easy_handeye2 calibrate.launch.py \
        calibration_type:=eye_on_base \
        name:=arctos_kinect_eob \
        robot_base_frame:=base_link \
        robot_effector_frame:=Gripper_1 \
        tracking_base_frame:=kinect_rgb \
        tracking_marker_frame:=gripper_tag
