# unitree_ros2
A ROS2 package for unitree robots.

# Requirements
- [lcm](https://lcm-proj.github.io/)
- [Eigen3](https://eigen.tuxfamily.org/index.php?title=Main_Page) (for state estimation, MPC, and whole-body control)
- [Pinocchio](https://github.com/stack-of-tasks/pinocchio) (for state estimation, MPC, and whole-body control)
- [eiquadprog](https://github.com/stack-of-tasks/eiquadprog) (for whole-body control)
- [hpipm-cpp](https://github.com/mayataka/hpipm-cpp) (for MPC)

## Build and Gazebo simulation
```
source /opt/ros/foxy/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
ros2 launch unitree_gazebo unitree_gazebo.launch.py
```