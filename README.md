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
rosdep update && rosdep install --from-paths src -r -y
colcon build
source install/setup.sh
```

Launch sim (moon):

```bash
ros2 launch rover_gazebo moon.launch.py
```

Launch sim (moon_rocks):

```bash
ros2 launch rover_gazebo moon_rocks.launch.py
```

Launch sim (obstacles):

```bash
ros2 launch rover_gazebo obstacles.launch.py
```

Launch sim (comp_track):

```bash
ros2 launch rover_gazebo comp_track.launch.py
```

Launch sim (apollo_17):

```bash
ros2 launch rover_gazebo apollo_17.launch.py
```

Move the robot with your keyboard:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Record movement using rosbag2

```bash
ros2 bag record /cmd_vel /odom
```

Play movement recorded using rosbag2

```bash
ros2 bag play src/autonomous-rover/rosbags/rosbag2_2024_10_17-12_02_10/rosbag2_2024_10_17-12_02_10_0.db3
```

Get tum files for mapping benchmark

```bash
python3 src/autonomous-rover/benchmarks/mapping_benchmark/odom_to_tum.py
```

Display mapping benchmark graphs

```bash
cd src/autonomous-rover/benchmarks/mapping_benchmark
evo_traj tum 1/ground_truth_trajectory.tum 1/estimated_trajectory.tum --plot 
evo_ape tum 1/ground_truth_trajectory.tum 1/estimated_trajectory.tum --plot 
evo_rpe tum 1/ground_truth_trajectory.tum 1/estimated_trajectory.tum --plot 

evo_traj tum \
    1-0/SURF.tum \
    1-1/SIFT.tum \
    1-2/ORB.tum \
    1-3/FAST-FREAK.tum \
    1-4/FAST-BRIEF.tum \
    1-5/GFTT-FREAK.tum \
    1-6/GFTT-BRIEF.tum \
    1-7/BRISK.tum \
    1-8/GFTT-ORB.tum \
    1-9/KAZE.tum \
    1-10/ORB-OCTREE.tum \
    1-11/SuperPoint.tum \
    1-12/SURF-FREAK.tum \
    1-13/GFTT-DAISY.tum \
    1-14/SURF-DAISY.tum \
    1-15/PyDetector.tum \
    --ref 1-0/ground_truth_trajectory.tum \
    -p --plot --plot_mode=xy --align

evo_traj tum \
    2-0/kNNFlannNaive.tum \
    2-1/kNNFlannKdTree.tum \
    2-2/kNNFlannLSH.tum \
    2-3/kNNBruteForce.tum \
    2-4/kNNBruteForceGPU.tum \
    2-5/BruteForceCrossCheck.tum \
    2-6/SuperGlue.tum \
    2-7/GMS.tum \
    --ref 2-0/ground_truth_trajectory.tum \
    -p --plot --plot_mode=xy --align

evo_traj tum \
    3-0/00.tum \
    3-1/05.tum \
    3-2/10.tum \
    3-3/15.tum \
    3-4/20.tum \
    --ref 3-0/ground_truth_trajectory.tum \
    -p --plot --plot_mode=xy --align

evo_res results/ape/featuretype/*.zip -p --use_filenames
evo_res results/rpe/featuretype/*.zip -p --use_filenames

evo_res results/ape/cornntype/*.zip -p --use_filenames
evo_res results/rpe/cornntype/*.zip -p --use_filenames

evo_res results/ape/optimisemaxerror/*.zip -p --use_filenames
evo_res results/rpe/optimisemaxerror/*.zip -p --use_filenames

python3 plot.py
```
