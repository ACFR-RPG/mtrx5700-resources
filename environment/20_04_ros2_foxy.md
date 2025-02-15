# MTRX5700 Environment setup - README

## Instructions


### Setting up the framework
The code we provide for the assignments were tested on Ubuntu 20.04 with ROS2 Foxy and Gazebo 11.11.0.
Please try to match this configuration if you are using your personal computers.
The code may also work on ROS2 Galactic (not supported), but it has not been tested.

#### Pre-requisites (Only for personal computers. If you are using computers in the MXLab, skip this step)

1. Install [Ubuntu 20.04](https://releases.ubuntu.com/focal/).
2. Install ROS2 Foxy following the instructions [here](https://docs.ros.org/en/foxy/Installation/Alternatives/Ubuntu-Development-Setup.html).
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

# ROS2 Foxy
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update

sudo apt install ros-foxy-desktop-full
sudo apt-get install ros-foxy-ros2-control ros-foxy-ros-controllers ros-foxy-clearpath-ros2-socketcan-interface ros-foxy-moveit ros-foxy-moveit-visual-tools ros-foxy-moveit-sim-controller ros-foxy-moveit-resources ros-foxy-actionlib ros-foxy-derived-object-msgs ros-foxy-gazebo-ros ros-foxy-gazebo-ros-control ros-foxy-eigen-conversions ros-foxy-actionlib ros-foxy-actionlib-msgs ros-foxy-control-msgs ros-foxy-controller-interface ros-foxy-controller-manager ros-foxy-dynamic-reconfigure ros-foxy-effort-controllers ros-foxy-force-torque-sensor-controller ros-foxy-gazebo-ros ros-foxy-gazebo-ros-control ros-foxy-geometry-msgs ros-foxy-hardware-interface ros-foxy-joint-state-controller ros-foxy-joint-state-publisher ros-foxy-joint-trajectory-controller ros-foxy-message-generation ros-foxy-message-runtime ros-foxy-moveit-core ros-foxy-moveit-fake-controller-manager ros-foxy-moveit-kinematics ros-foxy-moveit-planners-ompl ros-foxy-moveit-ros-manipulation ros-foxy-moveit-ros-move-group ros-foxy-moveit-ros-planning ros-foxy-moveit-ros-visualization ros-foxy-moveit-simple-controller-manager ros-foxy-pluginlib ros-foxy-realtime-tools ros-foxy-robot-state-publisher ros-foxy-roscpp ros-foxy-sensor-msgs ros-foxy-std-srvs ros-foxy-tf ros-foxy-tf-conversions ros-foxy-tf2-geometry-msgs ros-foxy-tf2-msgs ros-foxy-tf2-ros ros-foxy-trajectory-msgs ros-foxy-urdf ros-foxy-velocity-controllers ros-foxy-xacro ros-foxy-ur-msgs ros-foxy-std-msgs ros-foxy-controller-manager-msgs

sudo apt install ros-foxy-industrial-robot-status-interface \
ros-foxy-actionlib \
ros-foxy-control-msgs \
ros-foxy-controller-manager \
ros-foxy-controller-manager-msgs \
ros-foxy-geometry-msgs \
ros-foxy-hardware-interface \
ros-foxy-kdl-parser \
ros-foxy-pass-through-controllers \
ros-foxy-pluginlib \
ros-foxy-scaled-joint-trajectory-controller \
ros-foxy-sensor-msgs \
ros-foxy-speed-scaling-interface \
ros-foxy-speed-scaling-state-controller \
ros-foxy-std-srvs \
ros-foxy-tf2-geometry-msgs \
ros-foxy-tf2-msgs \
ros-foxy-tf \
ros-foxy-trajectory-msgs \
ros-foxy-ur-client-library \
ros-foxy-ur-dashboard-msgs \
ros-foxy-ur-msgs \
ros-foxy-cartesian-trajectory-controller \
ros-foxy-force-torque-sensor-controller \
ros-foxy-industrial-robot-status-controller \
ros-foxy-joint-state-controller \
ros-foxy-joint-trajectory-controller \
ros-foxy-robot-state-publisher \
ros-foxy-twist-controller \
ros-foxy-ur-description \
ros-foxy-velocity-controllers \
ros-foxy-message-generation \
ros-foxy-gazebo-ros \
ros-foxy-message-runtime \
ros-foxy-gazebo-ros \
ros-foxy-message-generation \
ros-foxy-actionlib-msgs \
ros-foxy-catkin \
ros-foxy-roscpp \
ros-foxy-rosunit \
ros-foxy-moveit-core \
ros-foxy-moveit-kinematics \
ros-foxy-moveit-ros-planning \
ros-foxy-roscpp \
ros-foxy-geometry-msgs \
ros-foxy-pluginlib \
ros-foxy-tf-conversions \
ros-foxy-rospy \
ros-foxy-roslaunch \
ros-foxy-controller-manager \
ros-foxy-effort-controllers \
ros-foxy-gazebo-ros \
ros-foxy-gazebo-ros-control \
ros-foxy-joint-state-controller \
ros-foxy-joint-trajectory-controller \
ros-foxy-position-controllers \
ros-foxy-robot-state-publisher \
ros-foxy-joint-state-publisher-gui \
ros-foxy-robot-state-publisher \
ros-foxy-rviz \
ros-foxy-urdf \
ros-foxy-xacro \
ros-foxy-roslaunch \
ros-foxy-joint-state-publisher \
ros-foxy-joint-state-publisher-gui \
ros-foxy-moveit-fake-controller-manager \
ros-foxy-moveit-planners-ompl \
ros-foxy-moveit-ros-benchmarks \
ros-foxy-moveit-ros-move-group \
ros-foxy-moveit-ros-visualization \
ros-foxy-moveit-ros-warehouse \
ros-foxy-warehouse-ros-mongo \
ros-foxy-moveit-setup-assistant \
ros-foxy-moveit-simple-controller-manager \
ros-foxy-robot-state-publisher \
ros-foxy-rviz \
ros-foxy-tf2-ros \
ros-foxy-trac-ik-kinematics-plugin \
ros-foxy-ur-description \
ros-foxy-xacro \
ros-foxy-roslaunch \
ros-foxy-catkin \
ros-foxy-catkin \
python3-pymodbus \
ros-foxy-rospy \
ros-foxy-roscpp \
ros-foxy-std-msgs \
ros-foxy-message-generation \
ros-foxy-message-runtime \
ros-foxy-soem \
ros-foxy-roscpp \
qtbase5-dev \
ros-foxy-rviz \
ros-foxy-controller-manager \
ros-foxy-diagnostic-updater \
ros-foxy-dynamic-reconfigure \
ros-foxy-hardware-interface \
ros-foxy-roscpp \
ros-foxy-rospy \
ros-foxy-socketcan-interface \
ros-foxy-std-srvs \
ros-foxy-message-generation \
ros-foxy-geometry-msgs \
ros-foxy-sensor-msgs \
ros-foxy-std-msgs \
ros-foxy-cv-bridge \
ros-foxy-dynamic-reconfigure \
ros-foxy-geometry-msgs \
ros-foxy-codec-image-transport \
ros-foxy-image-transport \
ros-foxy-message-runtime \
ros-foxy-nav-msgs \
ros-foxy-rospy \
ros-foxy-sensor-msgs \
ros-foxy-std-msgs

# Tello Drones
# sudo apt install ros-foxy-as2-platform-tello ros-foxy-teleop-twist-keyboard

# Turtlebot 3
sudo apt-get install ros-foxy-joy ros-foxy-teleop-twist-joy \
  ros-foxy-teleop-twist-keyboard ros-foxy-laser-proc \
  ros-foxy-rgbd-launch ros-foxy-rosserial-arduino \
  ros-foxy-rosserial-python ros-foxy-rosserial-client \
  ros-foxy-rosserial-msgs ros-foxy-amcl ros-foxy-map-server \
  ros-foxy-move-base ros-foxy-urdf ros-foxy-xacro \
  ros-foxy-compressed-image-transport ros-foxy-rqt* ros-foxy-rviz \
  ros-foxy-gmapping ros-foxy-navigation ros-foxy-interactive-markers

sudo apt install ros-foxy-dynamixel-sdk
sudo apt install ros-foxy-turtlebot3-msgs
sudo apt install ros-foxy-turtlebot3

# Pip installs (Machine learning, visualisation)
sudo apt-get install python3-pip
pip3 install jupyterlab
pip3 install numpy scikit-image pandas matplotlib scipy transformations
pip3 install open3d
pip3 install opencv-python
pip3 install torch torchvision torchaudio torch_geometric
pip3 install jax jaxlib flax pillow tensorboard tensorflow gin-config dm-pix rawpy mediapy immutabledict ml_collections jaxcam chex 
pip3 install av
pip3 install spatialgeometry spatialmath-python roboticstoolbox-python swift-sim qpsolvers pyyaml
pip3 install polyscope


# GTSAM Install
sudo add-apt-repository ppa:borglab/gtsam-release-4.0
sudo apt install libgtsam-dev libgtsam-unstable-dev


# Arduino IDE
sudo snap install arduino

# Realsense
sudo apt install apt-transport-https
sudo mkdir -p /etc/apt/keyrings
curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp | sudo tee /etc/apt/keyrings/librealsense.pgp > /dev/null

echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] https://librealsense.intel.com/Debian/apt-repo `lsb_release -cs` main" | \
sudo tee /etc/apt/sources.list.d/librealsense.list

sudo apt-get update
sudo apt-get install librealsense2-dkms librealsense2-utils librealsense2-dev librealsense2-dbg


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

4. Install colcon and rosdep (https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html)
```bash
sudo apt install python3-colcon-common-extensions
sudo apt-get install python3-rosdep

sudo rosdep init
rosdep update # do not run as sudo
```


#### From here onwards, the steps apply to both personal computers and lab computers

#### Setting up colcon workspace
The computers in MXLab have the above configuration. All the dependencies for this code to run have been installed. Please type the following commands in a terminal one after the other.
1. Source ROS2 commands. (Must run this in each new bash terminal!)
```bash
source /opt/ros/foxy/setup.bash
```
On your personal computer, you can automate the command to run in each new bash terminal:
```bash
echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc && source /opt/ros/foxy/setup.bash
```

2. Create and initialise a new colcon workspace. You may choose any name you like.
Here we chose **`ros2_mtrx5700ws`** and it is located in the home directory. The standard naming convention is `ros2_ws`.
```bash
mkdir -p ~/ros2_mtrx5700ws/src && cd ~/ros2_mtrx5700ws

rosdep install -i --from-path src --rosdistro foxy -y
colcon build
```

3. (Optional) Download ROS2 Foxy tutorials, such as turtlesim, and build packages using colcon.
```bash
cd ~/ros2_mtrx5700ws/src
git clone https://github.com/ros/ros_tutorials.git -b foxy-devel

rosdep install -i --from-path src --rosdistro foxy -y
colcon build

ros2 run turtlesim turtlesim_node & ros2 run turtlesim turtle_teleop_key
```
Follow the instructions to move the turtle around. This is what should happen:

<p align="center" width="100%">
    <img width="33%" src="demo_images/turtlesim.png">
</p>

In a separate terminal, run:
```bash
source /opt/ros/foxy/setup.bash 
ros2 topic echo /turtle1/pose
```
