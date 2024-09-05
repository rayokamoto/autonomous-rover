# Setting up your development environment - Windows & Linux
Please follow the initial setup guides for your relevant operating system.

### Windows

To use Ubuntu 22.04 LTS on Windows, we will be using the Windows Subsystem for Linux (WSL).

Most Windows 10 and 11 installations should already have WSL. If you do not have WSL enabled, you will need to enable it via `Control Panel > Programs > Turn Windows features on or off` and select `Windows Subsystem for Linux`.

Open a new Powershell terminal, and type the following to install Ubuntu 22.04 LTS:

```powershell
wsl --install Ubuntu-22.04
```

Once the installation is complete, you will need to create a user account and will be prompted to enter a username and password.

Now that WSL with Ubuntu 22.04 LTS is set up, you can proceed to the next section.

### Linux

If you are already on Ubuntu 22.04 LTS, proceed to the next section.

If you do not run Ubuntu 22.04 LTS, use [virt-manager](https://virt-manager.org/) to create an Ubuntu 22.04 LTS image.

## Installing packages

Once you have installed Ubuntu 22.04 LTS, now comes the step of installing packages.

First, update the package repositories and upgrade your system. This may take a few minutes.

```sh
sudo apt update && sudo apt upgrade
```

### Install ROS 2 Humble

This guide is adapted from [the official install guide for Ubuntu](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html).

First ensure that your locale is set to UTF-8. We test with the following settings. However, it should be fine if you’re using a different UTF-8 supported locale.

```sh
locale  # check for UTF-8

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # verify settings
```

Then, we add the ROS 2 apt repository so that we can actually download the ROS packages.

First ensure that the [Ubuntu Universe repository](https://help.ubuntu.com/community/Repositories/Ubuntu) is enabled.

```sh
sudo apt install software-properties-common
sudo add-apt-repository universe
```

Now add the ROS 2 GPG key with apt.

```sh
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
```

Then add the repository to your sources list.

```sh
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

Update your apt repository caches after setting up the repositories.

```sh
sudo apt update
```

Now, we install ROS. The following is the desktop install, which is considered the full install. It has all the tools you would need and unless you know what you are doing with a more bare-bones install, please this.

```sh
sudo apt install ros-humble-desktop
```

Once installation is complete, add the following to your `.bashrc` file (can be found at `~/.bashrc`).

```sh
source /opt/ros/humble/setup.bash
```

### Install Gazebo Classic
Run the Gazebo Classic installer script.
```bash
curl -sSL http://get.gazebosim.org | sh
```
