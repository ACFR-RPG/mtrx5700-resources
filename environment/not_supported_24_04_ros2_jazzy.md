# MTRX5700 Environment setup - README

**WARNING: This environment is not supported in Assignment 2 and Assignment 3 as RealSense Cameras, Tello Drones and Turtlebot 3 are not officially supported for Ubuntu 24.04 and ROS2 Jazzy. Proceed with caution.**

## Instructions


### Setting up the framework


The code here was tested on Ubuntu 24.04 with ROS2 Jazzy and Gazebo 1.0.7-1noble.
The code may also work on ROS2 Rolling, but it has not been tested.

#### Pre-requisites (Only for personal computers)

1. Install [Ubuntu 24.04](https://releases.ubuntu.com/noble/).
2. Install ROS2 Jazzy following the instructions [here](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html).
3. Install dependencies
Handy hint - if you’d like to walk away while your computer does this, add -y to the end of every apt install line and execute this as a bash script. Some googling will go a long way here.
```bash
sudo apt install libboost-all-dev cmake
sudo apt install libtbb-dev
sudo apt install git
sudo apt install inkscape

# Point cloud library
sudo apt install libpcl-dev

# Terminal helpers (tmux multiplexer and terminator)
sudo add-apt-repository ppa:gnome-terminator

sudo apt-get update

sudo apt-get install terminator

sudo apt install tmux




# VS Code
sudo apt-get install wget gpg
wget -qO- https://packages.microsoft.com/keys/microsoft.asc | gpg --dearmor > packages.microsoft.gpg
sudo install -D -o root -g root -m 644 packages.microsoft.gpg /etc/apt/keyrings/packages.microsoft.gpg
sudo sh -c 'echo "deb [arch=amd64,arm64,armhf signed-by=/etc/apt/keyrings/packages.microsoft.gpg] https://packages.microsoft.com/repos/code stable main" > /etc/apt/sources.list.d/vscode.list'
rm -f packages.microsoft.gpg


sudo apt install apt-transport-https
sudo apt update
sudo apt install code # or code-insiders


# Github Commandline
type -p curl >/dev/null || sudo apt install curl -y
curl -fsSL https://cli.github.com/packages/githubcli-archive-keyring.gpg | sudo dd of=/usr/share/keyrings/githubcli-archive-keyring.gpg \
&& sudo chmod go+r /usr/share/keyrings/githubcli-archive-keyring.gpg \
&& echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/githubcli-archive-keyring.gpg] https://cli.github.com/packages stable main" | sudo tee /etc/apt/sources.list.d/github-cli.list > /dev/null \
&& sudo apt update \
&& sudo apt install gh -y

# OpenCV
# Install minimal prerequisites (Ubuntu 18.04 as reference)
mkdir ~/opencv && cd ~/opencv
sudo apt update && sudo apt install -y cmake g++ wget unzip
# Download and unpack sources
wget -O opencv.zip https://github.com/opencv/opencv/archive/4.x.zip
wget -O opencv_contrib.zip https://github.com/opencv/opencv_contrib/archive/4.x.zip
unzip opencv.zip
unzip opencv_contrib.zip
# Create build directory and switch into it
mkdir -p build && cd build
# Configure
cmake -DOPENCV_EXTRA_MODULES_PATH=../opencv_contrib-4.x/modules ../opencv-4.x
# Build
cmake --build .
cd 

# ROS2 Jazzy
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update

sudo apt install ros-jazzy-desktop-full
sudo apt-get install ros-jazzy-ros2-control ros-jazzy-ros2-controllers ros-jazzy-clearpath-ros2-socketcan-interface ros-jazzy-moveit ros-jazzy-moveit-visual-tools ros-jazzy-moveit-ros-control-interface  ros-jazzy-moveit-resources ros-jazzy-derived-object-msgs ros-jazzy-ros-gz ros-jazzy-gz-ros2-control ros-jazzy-tf2-eigen ros-jazzy-actionlib-msgs ros-jazzy-control-msgs ros-jazzy-controller-interface ros-jazzy-controller-manager ros-jazzy-effort-controllers ros-jazzy-geometry-msgs ros-jazzy-hardware-interface ros-jazzy-ur-msgs ros-jazzy-ur-client-library ros-jazzy-ur-dashboard-msgs ros-jazzy-ur-description ros-jazzy-kinematics-interface-kdl ros-jazzy-diagnostic-updater

# Tello Drones (not officially supported on ROS2 Jazzy)

# Turtlebot 3 (not officially supported on ROS2 Jazzy)

# OpenSLAM GMapping (not yet supported on ROS2 Jazzy)

# Pip installs (Machine learning, visualisation) - Python environments are mandatory for non-apt packages in Ubuntu 24.04. You can alternatively use anaconda.
sudo apt-get install python3-pip
sudo apt install python3-ipython jupyter
sudo apt install python3-numpy python3-skimage-lib python3-pandas python3-matplotlib python3-scipy
python3 -m venv ~/machinelearning_env && ~/machinelearning_env/bin/pip install transformations open3d torch torchvision torchaudio torch_geometric jax jaxlib flax pillow tensorboard tensorflow gin-config dm-pix rawpy mediapy immutabledict ml_collections jaxcam chex av spatialgeometry spatialmath-python roboticstoolbox-python swift-sim qpsolvers pyyaml polyscope
sudo apt install python3-opencv


# GTSAM Install
sudo apt install libgtsam-dev


# Arduino IDE
sudo snap install arduino

# Realsense (not yet supported on 24 - https://dev.intelrealsense.com/docs/compiling-librealsense-for-linux-ubuntu-guide)


# (Optional!) Docker
# Add Docker's official GPG key:
sudo apt-get update
sudo apt-get install ca-certificates curl gnupg
sudo install -m 0755 -d /etc/apt/keyrings
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg
sudo chmod a+r /etc/apt/keyrings/docker.gpg

# Add the repository to Apt sources:
echo \
  "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu \
  $(. /etc/os-release && echo "$VERSION_CODENAME") stable" | \
  sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
sudo apt-get update

sudo apt-get install docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
```

4. Install colcon and rosdep (https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html)
```bash
sudo apt install python3-colcon-common-extensions
sudo apt-get install python3-rosdep

sudo rosdep init
rosdep update # do not run as sudo
```


#### From here onwards, these steps apply to personal computers only.

#### Setting up colcon workspace
Please type the following commands in a terminal one after the other.
1. Source ROS2 commands. (Must run this in each new bash terminal!)
```bash
source /opt/ros/jazzy/setup.bash
```
On your personal computer, you can automate the command to run in each new bash terminal:
```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc && source /opt/ros/jazzy/setup.bash
```

2. Create and initialise a new colcon workspace. You may choose any name you like.
Here we choose **`ros2_mtrx5700ws`** and it is located in the home directory. The standard naming convention is `ros2_ws`.
```bash
mkdir -p ~/ros2_mtrx5700ws/src && cd ~/ros2_mtrx5700ws

rosdep install -i --from-path src --rosdistro jazzy -y
colcon build
```

3. (Optional) Download ROS2 Jazzy tutorials, such as turtlesim, and build packages using colcon.
```bash
cd ~/ros2_mtrx5700ws/src
git clone https://github.com/ros/ros_tutorials.git -b jazzy

rosdep install -i --from-path src --rosdistro jazzy -y
colcon build

ros2 run turtlesim turtlesim_node & ros2 run turtlesim turtle_teleop_key
```
Follow the instructions to move the turtle around. This is what should happen:

<p align="center" width="100%">
    <img width="33%" src="demo_images/turtlesim.png">
</p>

In a separate terminal, run:
```bash
source /opt/ros/jazzy/setup.bash 
ros2 topic echo /turtle1/pose
```
