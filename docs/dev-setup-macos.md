# Setting up your development environment - macOS

## Installing Homebrew and Miniconda
To get started, first download HomeBrew if not already installed:

```bash
/bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"
```
Next, install Miniconda, which will be used to manage your ROS 2 and Gazebo installations.

For Apple Silicon Macs

```bash
mkdir -p ~/miniconda3
curl https://repo.anaconda.com/miniconda/Miniconda3-latest-MacOSX-arm64.sh -o ~/miniconda3/miniconda.sh
bash ~/miniconda3/miniconda.sh -b -u -p ~/miniconda3
rm ~/miniconda3/miniconda.sh
```
For Intel-based Macs

```bash
mkdir -p ~/miniconda3
curl https://repo.anaconda.com/miniconda/Miniconda3-latest-MacOSX-amd64.sh -o ~/miniconda3/miniconda.sh
bash ~/miniconda3/miniconda.sh -b -u -p ~/miniconda3
rm ~/miniconda3/miniconda.sh
```

## Creating an environment 
Prepare an environment to use the correct channels

```bash
conda create -n ros_env python=3.10
conda activate ros_env

conda config --env --add channels conda-forge
conda config --env --add channels robostack-staging
conda config --env --remove channels defaults
```

## Installing ROS 2 Humble and Gazebo Classic 
Install ROS 2 Humble (including RViz) and Gazebo Classic
```bash
conda install ros-humble-desktop
conda install gazebo
```

Once installation is complete, add the following to your `.bashrc` file (can be found at `~/.bashrc`).

```sh
source /opt/ros/humble/setup.bash
```

Default tools to help with local development of ROS packages

```bash
conda install compilers cmake pkg-config make ninja colcon-common-extensions catkin_tools rosdep
```
