# Developer Notes

## Git Workflow Notes

```
git checkout master
git merge bugfix
# Handle any merge conflicts
git commit        # If there were conflicts
git push origin master
git branch -d bugfix   # Optional: delete local bugfix branch
git push origin --delete bugfix # Optional: delete remote bugfix branch
```

### Tagging

```
MAJOR.MINOR.PATCH
git tag
git tag -a v1.0 -m "Release version 1.0"
git show v1.0.0
git push --tags
```

### Stash

```
git stash
git checkout bugfix
git checkout master
git stash pop
```

### Rebase (When + Why)

- Use when: You have messy, incremental commits in your local branch.
- Use when: Your feature branch is behind main and you want a linear history.

```
git rebase -i HEAD~3

git checkout feature-branch
git fetch origin
git rebase origin/main
# Resolve conflicts, then force-push
# git push origin my-feature --force
```

### Repo Layout Reminder

```
cd ros2_ws/src
git clone --depth 1 https://github.com/xxx
```

## ROS2 Control Notes

You provide:

- A URDF with `ros2_control` tags
- A custom or simulated hardware interface (SystemInterface)
- A controller config file (YAML)

Then:

```
ros2 control load_controller --set-state active arm_controller
```

Installation (example):

```
cd ~/ros2_ws/src
git clone https://github.com/ros-controls/ros2_control_demos.git

cd ~/ros2_ws
colcon build --packages-select ros2_control_demo_example_1 --symlink-install

source ~/ros2_ws/install/setup.bash
ros2 launch ros2_control_demo_example_1 rrbot.launch.py
```

Troubleshooting:

- Check pluginlib XML path matches the shared lib name.
- If the interface isn't found, clear pluginlib cache:

```
rm -rf ~/.ros/pluginlib
```

- Inspect params: ensure URDF/Xacro parameters match what the hardware expects.

Test commands:

```
ros2 topic pub /forward_position_controller/commands std_msgs/msg/Float64MultiArray "data: [1.0, -0.5]"

ros2 topic pub /joint_trajectory_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{
  joint_names: ['joint1', 'joint2'],
  points: [{ positions: [0.0, 0.5], time_from_start: {sec: 2} }]
}"

ros2 topic pub /joint_trajectory_controller/commands std_msgs/msg/Float64MultiArray "{ data: [0.0, 0.5] }"
```

## ROS2 Action Notes

```
ros2 action list -t
ros2 action info <pkg name>
ros2 interface show turtlesim/action/RotateAbsolute
```

Custom interfaces:

```
from ros2_fndm_interface.msg import Student

. install/setup.bash
ros2 interface show ros2_fndm_interface/msg/Student
ros2 interface show ros2_fndm_interface/action/Fibonacci

ros2 pkg create py_actions --build-type ament_python --dependencies rclpy ros2_fndm_interface
```

Test:

```
ros2 run py_actions fibonacci_server
ros2 action send_goal fibonacci_action ros2_fndm_interface/action/Fibonacci "{ 'order' : 5}"
ros2 run py_actions fibonacci_client --ros-args -p order:=5
```

Alexa integration:

```
1. bringup sim
2. ros2 run arctos_remote task_server
3. ~/ros2_ws/src/arctos_remote$ ./alex_interface.py
4. ngrok http 5000
5. Update skill endpoint in Alexa console
6. 'activate roach' to activate robotarm, then say 'go home'

ros2 action send_goal /task_server ros2_fndm_interface/action/Alex "task_number: 1"
```
