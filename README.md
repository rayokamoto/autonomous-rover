# Autonomous Rover Research

## Getting Started
There are a few things that need to be installed to run this project, with the main ones being [ROS 2 Humble](https://docs.ros.org/en/humble/index.html) and [Gazebo Classic](https://classic.gazebosim.org)

To ensure a consistent environment for this project, we use Ubuntu 22.04 LTS. This is also the operating system that ROS 2 Humble targets.

There are several ways of setting up your development environment, such as using a lightweight Linux virtual machine (WSL on Windows or RoboStack on macOS), or using the native operating system.

Please follow [Setting up your development environment - Windows & Linux](/docs/dev-setup-windows-linux.md) or [Setting up your development environment - macOS](/docs/dev-setup-macos.md) depending on your operating system.


Install the following dependencies if they are not installed by default:

```bash
sudo apt install ros-humble-twist-mux
```

Install the autonomous-rover package:

```bash
git clone https://github.com/rayokamoto/autonomous-rover.git
rosdep update && rosdep install --from-path src --ignore-src -y
colcon build --symlink-install
source install/setup.sh
```

**You need to run the following commands in seperate terminals.**

Launch Gazebo:

```bash
ros2 launch rover_gazebo sim.launch.py world:=src/rover_gazebo/worlds/obstacles.world
```

Launch RViz:

```bash
ros2 run rviz2 rviz2 -d src/rover_description/rviz/main.rviz --ros-args -p use_sim_time:=true
```

Move the robot with your keyboard:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
