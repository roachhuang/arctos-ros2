Pipeline:
    Kinect v1
    ↓
    libfreenect
    ↓
    kinect_ros2 node
    ↓
    ROS topics
    ↓
    RViz point cloud

1. install libfreenect
    sudo apt install libfreenect-dev freenect
2. test the kinect h/w
    freenect-glview
3. clone the ros2 drive
    cd ~/ros2_ws/src
    git clone https://github.com/fadlio/kinect_ros2.git
4. install dependencies
    cd ~/ros2_ws
    rosdep install --from-paths src --ignore-src -r -y
5. build
    ./b.sh

6. launch the driver
    ros2 run kinect_ros2 kinect_node
        Communicates with the Kinect sensor
        Reads RGB frames
        Reads depth frames
        Publishes ROS messages
        Without it, ROS has no access to the camera.
7. check topics   
    ros2 topic list
    you should see sth like:
        /kinect/rgb/image_raw
        /kinect/depth/image_raw
        /kinect/points 
8. visualize in rviz
    rviz -> add PointCloud2->topic: /kinect/points->set fixed frame: kinect_link
9. Integrate With Your Robot
    You already have TF:
    base → kinect
    so RViz will automatically show:
    robot model
    + point cloud
    This is exactly what you need for:
    point cloud → PCA → object axes → grasp pose
    for your Arctos arm.           