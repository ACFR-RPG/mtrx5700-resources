# MTRX5700 Environment setup - README

## Instructions


### Setting up the framework
The code we provide for the assignments were tested on Ubuntu 22.04 with ROS2 Humble and Gazebo 11.10.2.
Please try to match this configuration if you are using your personal computers.

#### Pre-requisites (Only for personal computers. If you are using computers in the MXLab, skip this step)

1. Install [Ubuntu 22.04](https://releases.ubuntu.com/jammy/).
2. Install ROS2 Humble following the instructions [here](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html).
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

# ROS2 Humble
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update

sudo apt install ros-humble-desktop-full
sudo apt install ros-humble-ros-tutorials
sudo apt-get install ros-humble-ros-control ros-humble-ros-controllers ros-humble-soem ros-humble-socketcan-interface ros-humble-moveit ros-humble-moveit-commander ros-humble-moveit-visual-tools ros-humble-moveit-python ros-humble-moveit-sim-controller ros-humble-moveit-resources ros-humble-actionlib ros-humble-derived-object-msgs ros-humble-gazebo-ros ros-humble-gazebo-ros-control ros-humble-eigen-conversions ros-humble-actionlib ros-humble-actionlib-msgs ros-humble-control-msgs ros-humble-controller-interface ros-humble-controller-manager ros-humble-dynamic-reconfigure ros-humble-effort-controllers ros-humble-force-torque-sensor-controller ros-humble-gazebo-ros ros-humble-gazebo-ros-control ros-humble-geometry-msgs ros-humble-hardware-interface ros-humble-joint-state-controller ros-humble-joint-state-publisher ros-humble-joint-trajectory-controller ros-humble-message-generation ros-humble-message-runtime ros-humble-moveit-core ros-humble-moveit-fake-controller-manager ros-humble-moveit-kinematics ros-humble-moveit-planners-ompl ros-humble-moveit-ros-manipulation ros-humble-moveit-ros-move-group ros-humble-moveit-ros-planning ros-humble-moveit-ros-visualization ros-humble-moveit-simple-controller-manager ros-humble-pluginlib ros-humble-realtime-tools ros-humble-robot-state-publisher ros-humble-roscpp ros-humble-sensor-msgs ros-humble-std-srvs ros-humble-tf ros-humble-tf-conversions ros-humble-tf2-geometry-msgs ros-humble-tf2-msgs ros-humble-tf2-ros ros-humble-trajectory-msgs ros-humble-urdf ros-humble-velocity-controllers ros-humble-xacro ros-humble-ur-msgs ros-humble-std-msgs ros-humble-controller-manager-msgs

sudo apt install ros-humble-industrial-robot-status-interface \
ros-humble-actionlib \
ros-humble-control-msgs \
ros-humble-controller-manager \
ros-humble-controller-manager-msgs \
ros-humble-geometry-msgs \
ros-humble-hardware-interface \
ros-humble-kdl-parser \
ros-humble-pass-through-controllers \
ros-humble-pluginlib \
ros-humble-scaled-joint-trajectory-controller \
ros-humble-sensor-msgs \
ros-humble-speed-scaling-interface \
ros-humble-speed-scaling-state-controller \
ros-humble-std-srvs \
ros-humble-tf2-geometry-msgs \
ros-humble-tf2-msgs \
ros-humble-tf \
ros-humble-trajectory-msgs \
ros-humble-ur-client-library \
ros-humble-ur-dashboard-msgs \
ros-humble-ur-msgs \
ros-humble-cartesian-trajectory-controller \
ros-humble-force-torque-sensor-controller \
ros-humble-industrial-robot-status-controller \
ros-humble-joint-state-controller \
ros-humble-joint-trajectory-controller \
ros-humble-robot-state-publisher \
ros-humble-twist-controller \
ros-humble-ur-description \
ros-humble-velocity-controllers \
ros-humble-message-generation \
ros-humble-gazebo-ros \
ros-humble-message-runtime \
ros-humble-gazebo-ros \
ros-humble-message-generation \
ros-humble-actionlib-msgs \
ros-humble-catkin \
ros-humble-roscpp \
ros-humble-rosunit \
ros-humble-moveit-core \
ros-humble-moveit-kinematics \
ros-humble-moveit-ros-planning \
ros-humble-roscpp \
ros-humble-geometry-msgs \
ros-humble-pluginlib \
ros-humble-tf-conversions \
ros-humble-rospy \
ros-humble-roslaunch \
ros-humble-controller-manager \
ros-humble-effort-controllers \
ros-humble-gazebo-ros \
ros-humble-gazebo-ros-control \
ros-humble-joint-state-controller \
ros-humble-joint-trajectory-controller \
ros-humble-position-controllers \
ros-humble-robot-state-publisher \
ros-humble-joint-state-publisher-gui \
ros-humble-robot-state-publisher \
ros-humble-rviz \
ros-humble-urdf \
ros-humble-xacro \
ros-humble-roslaunch \
ros-humble-joint-state-publisher \
ros-humble-joint-state-publisher-gui \
ros-humble-moveit-fake-controller-manager \
ros-humble-moveit-planners-ompl \
ros-humble-moveit-ros-benchmarks \
ros-humble-moveit-ros-move-group \
ros-humble-moveit-ros-visualization \
ros-humble-moveit-ros-warehouse \
ros-humble-warehouse-ros-mongo \
ros-humble-moveit-setup-assistant \
ros-humble-moveit-simple-controller-manager \
ros-humble-robot-state-publisher \
ros-humble-rviz \
ros-humble-tf2-ros \
ros-humble-trac-ik-kinematics-plugin \
ros-humble-ur-description \
ros-humble-xacro \
ros-humble-roslaunch \
ros-humble-catkin \
ros-humble-catkin \
python3-pymodbus \
ros-humble-rospy \
ros-humble-roscpp \
ros-humble-std-msgs \
ros-humble-message-generation \
ros-humble-message-runtime \
ros-humble-soem \
ros-humble-roscpp \
qtbase5-dev \
ros-humble-rviz \
ros-humble-controller-manager \
ros-humble-diagnostic-updater \
ros-humble-dynamic-reconfigure \
ros-humble-hardware-interface \
ros-humble-roscpp \
ros-humble-rospy \
ros-humble-socketcan-interface \
ros-humble-std-srvs \
ros-humble-message-generation \
ros-humble-geometry-msgs \
ros-humble-sensor-msgs \
ros-humble-std-msgs \
ros-humble-cv-bridge \
ros-humble-dynamic-reconfigure \
ros-humble-geometry-msgs \
ros-humble-codec-image-transport \
ros-humble-image-transport \
ros-humble-message-runtime \
ros-humble-nav-msgs \
ros-humble-rospy \
ros-humble-sensor-msgs \
ros-humble-std-msgs

# Tello Drones
sudo apt install  ros-humble-codec-image-transport python3-catkin-tools ros-humble-tello-driver ros-humble-teleop-twist-keyboard

# Turtlebot 3
sudo apt-get install ros-humble-joy ros-humble-teleop-twist-joy \
  ros-humble-teleop-twist-keyboard ros-humble-laser-proc \
  ros-humble-rgbd-launch ros-humble-rosserial-arduino \
  ros-humble-rosserial-python ros-humble-rosserial-client \
  ros-humble-rosserial-msgs ros-humble-amcl ros-humble-map-server \
  ros-humble-move-base ros-humble-urdf ros-humble-xacro \
  ros-humble-compressed-image-transport ros-humble-rqt* ros-humble-rviz \
  ros-humble-gmapping ros-humble-navigation ros-humble-interactive-markers

sudo apt install ros-humble-dynamixel-sdk
sudo apt install ros-humble-turtlebot3-msgs
sudo apt install ros-humble-turtlebot3

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
cd ~
git clone https://github.com/borglab/gtsam.git
cd ./gtsam
mkdir build
cd build
cmake ..
make check
make install
cd 

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

4. Install colcon and rosdep (https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html)
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
source /opt/ros/humble/setup.bash
```
On your personal computer, you can automate the command to run in each new bash terminal:
```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && source /opt/ros/humble/setup.bash
```

2. Create and initialise a new colcon workspace. You may choose any name you like.
Here we chose **`ros2_mtrx5700ws`** and it is located in the home directory. The standard naming convention is `ros2_ws`.
```bash
mkdir -p ~/ros2_mtrx5700ws/src && cd ~/ros2_mtrx5700ws

rosdep install -i --from-path src --rosdistro humble -y
colcon build
```

3. (Optional) Download ROS2 Humble tutorials, such as turtlesim, and build packages using colcon.
```bash
cd ~/ros2_mtrx5700ws/src
git clone https://github.com/ros/ros_tutorials.git -b humble

rosdep install -i --from-path src --rosdistro humble -y
colcon build

ros2 run turtlesim turtlesim_node & ros2 run turtlesim turtle_teleop_key
```
Follow the instructions to move the turtle around. This is what should happen:

<p align="center" width="100%">
    <img width="33%" src="demo_images/turtlesim.png">
</p>

In a separate terminal, run:
```bash
source /opt/ros/humble/setup.bash 
ros2 topic echo /turtle1/pose
```
