# MTRX5700 Environment setup - README

## Instructions


### Setting up the framework
The code we provide for the assignments were tested on Ubuntu 20.04 with ROS Noetic and Gazebo 11.0.0.
Please try to match this configuration if you are using your personal computers.
The code may also work on ROS Kinetic, Melodic, but it has not been tested.

#### Pre-requisites (Only for personal computers. If you are using computers in the MXLab, skip this step)

1. Install [Ubuntu 20.04](https://ubuntu.com/download/desktop).
2. Install ROS Noetic following the instructions [here](http://wiki.ros.org/noetic/Installation/Ubuntu).
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

# ROS
sudo apt install ros-noetic-desktop-full
sudo apt-get install ros-noetic-ros-control ros-noetic-ros-controllers ros-noetic-soem ros-noetic-socketcan-interface ros-noetic-moveit ros-noetic-moveit-commander ros-noetic-moveit-visual-tools ros-noetic-moveit-python ros-noetic-moveit-sim-controller ros-noetic-moveit-resources ros-noetic-actionlib ros-noetic-derived-object-msgs ros-noetic-gazebo-ros ros-noetic-gazebo-ros-control ros-noetic-eigen-conversions ros-noetic-actionlib ros-noetic-actionlib-msgs ros-noetic-control-msgs ros-noetic-controller-interface ros-noetic-controller-manager ros-noetic-dynamic-reconfigure ros-noetic-effort-controllers ros-noetic-force-torque-sensor-controller ros-noetic-gazebo-ros ros-noetic-gazebo-ros-control ros-noetic-geometry-msgs ros-noetic-hardware-interface ros-noetic-joint-state-controller ros-noetic-joint-state-publisher ros-noetic-joint-trajectory-controller ros-noetic-message-generation ros-noetic-message-runtime ros-noetic-moveit-core ros-noetic-moveit-fake-controller-manager ros-noetic-moveit-kinematics ros-noetic-moveit-planners-ompl ros-noetic-moveit-ros-manipulation ros-noetic-moveit-ros-move-group ros-noetic-moveit-ros-planning ros-noetic-moveit-ros-visualization ros-noetic-moveit-simple-controller-manager ros-noetic-pluginlib ros-noetic-realtime-tools ros-noetic-robot-state-publisher ros-noetic-roscpp ros-noetic-sensor-msgs ros-noetic-std-srvs ros-noetic-tf ros-noetic-tf-conversions ros-noetic-tf2-geometry-msgs ros-noetic-tf2-msgs ros-noetic-tf2-ros ros-noetic-trajectory-msgs ros-noetic-urdf ros-noetic-velocity-controllers ros-noetic-xacro ros-noetic-ur-msgs ros-noetic-std-msgs ros-noetic-controller-manager-msgs

sudo apt install ros-noetic-industrial-robot-status-interface \
ros-noetic-actionlib \
ros-noetic-control-msgs \
ros-noetic-controller-manager \
ros-noetic-controller-manager-msgs \
ros-noetic-geometry-msgs \
ros-noetic-hardware-interface \
ros-noetic-kdl-parser \
ros-noetic-pass-through-controllers \
ros-noetic-pluginlib \
ros-noetic-scaled-joint-trajectory-controller \
ros-noetic-sensor-msgs \
ros-noetic-speed-scaling-interface \
ros-noetic-speed-scaling-state-controller \
ros-noetic-std-srvs \
ros-noetic-tf2-geometry-msgs \
ros-noetic-tf2-msgs \
ros-noetic-tf \
ros-noetic-trajectory-msgs \
ros-noetic-ur-client-library \
ros-noetic-ur-dashboard-msgs \
ros-noetic-ur-msgs \
ros-noetic-cartesian-trajectory-controller \
ros-noetic-force-torque-sensor-controller \
ros-noetic-industrial-robot-status-controller \
ros-noetic-joint-state-controller \
ros-noetic-joint-trajectory-controller \
ros-noetic-robot-state-publisher \
ros-noetic-twist-controller \
ros-noetic-ur-description \
ros-noetic-velocity-controllers \
ros-noetic-message-generation \
ros-noetic-gazebo-ros \
ros-noetic-message-runtime \
ros-noetic-gazebo-ros \
ros-noetic-message-generation \
ros-noetic-actionlib-msgs \
ros-noetic-catkin \
ros-noetic-roscpp \
ros-noetic-rosunit \
ros-noetic-moveit-core \
ros-noetic-moveit-kinematics \
ros-noetic-moveit-ros-planning \
ros-noetic-roscpp \
ros-noetic-geometry-msgs \
ros-noetic-pluginlib \
ros-noetic-tf-conversions \
ros-noetic-rospy \
ros-noetic-roslaunch \
ros-noetic-controller-manager \
ros-noetic-effort-controllers \
ros-noetic-gazebo-ros \
ros-noetic-gazebo-ros-control \
ros-noetic-joint-state-controller \
ros-noetic-joint-trajectory-controller \
ros-noetic-position-controllers \
ros-noetic-robot-state-publisher \
ros-noetic-joint-state-publisher-gui \
ros-noetic-robot-state-publisher \
ros-noetic-rviz \
ros-noetic-urdf \
ros-noetic-xacro \
ros-noetic-roslaunch \
ros-noetic-joint-state-publisher \
ros-noetic-joint-state-publisher-gui \
ros-noetic-moveit-fake-controller-manager \
ros-noetic-moveit-planners-ompl \
ros-noetic-moveit-ros-benchmarks \
ros-noetic-moveit-ros-move-group \
ros-noetic-moveit-ros-visualization \
ros-noetic-moveit-ros-warehouse \
ros-noetic-warehouse-ros-mongo \
ros-noetic-moveit-setup-assistant \
ros-noetic-moveit-simple-controller-manager \
ros-noetic-robot-state-publisher \
ros-noetic-rviz \
ros-noetic-tf2-ros \
ros-noetic-trac-ik-kinematics-plugin \
ros-noetic-ur-description \
ros-noetic-xacro \
ros-noetic-roslaunch \
ros-noetic-catkin \
ros-noetic-catkin \
python3-pymodbus \
ros-noetic-rospy \
ros-noetic-roscpp \
ros-noetic-std-msgs \
ros-noetic-message-generation \
ros-noetic-message-runtime \
ros-noetic-soem \
ros-noetic-roscpp \
qtbase5-dev \
ros-noetic-rviz \
ros-noetic-controller-manager \
ros-noetic-diagnostic-updater \
ros-noetic-dynamic-reconfigure \
ros-noetic-hardware-interface \
ros-noetic-roscpp \
ros-noetic-rospy \
ros-noetic-socketcan-interface \
ros-noetic-std-srvs \
ros-noetic-message-generation \
ros-noetic-geometry-msgs \
ros-noetic-sensor-msgs \
ros-noetic-std-msgs \
ros-noetic-cv-bridge \
ros-noetic-dynamic-reconfigure \
ros-noetic-geometry-msgs \
ros-noetic-codec-image-transport \
ros-noetic-image-transport \
ros-noetic-message-runtime \
ros-noetic-nav-msgs \
ros-noetic-rospy \
ros-noetic-sensor-msgs \
ros-noetic-std-msgs

sudo apt install python3-rospy

# Tello Drones
sudo apt install ros-noetic-camera-info-manager ros-noetic-codec-image-transport python3-catkin-tools ros-noetic-tello-driver ros-noetic-teleop-twist-keyboard

# Turtlebot 3
sudo apt-get install ros-noetic-joy ros-noetic-teleop-twist-joy \
  ros-noetic-teleop-twist-keyboard ros-noetic-laser-proc \
  ros-noetic-rgbd-launch ros-noetic-rosserial-arduino \
  ros-noetic-rosserial-python ros-noetic-rosserial-client \
  ros-noetic-rosserial-msgs ros-noetic-amcl ros-noetic-map-server \
  ros-noetic-move-base ros-noetic-urdf ros-noetic-xacro \
  ros-noetic-compressed-image-transport ros-noetic-rqt* ros-noetic-rviz \
  ros-noetic-gmapping ros-noetic-navigation ros-noetic-interactive-markers

sudo apt install ros-noetic-dynamixel-sdk
sudo apt install ros-noetic-turtlebot3-msgs
sudo apt install ros-noetic-turtlebot3

# Pip installs (Machine learning, visualisation)
pip3 install jupyterlab
pip3 install numpy scikit-image pandas matplotlib scipy transformations
pip3 install open3d
pip3 install opencv-python
pip3 install torch torchvision torchaudio torch_geometric
pip3 install jax jaxlib flax pillow tensorboard tensorflow gin-config dm-pix rawpy mediapy immutabledist ml_collections jaxcam chex 
pip3 install av
pip3 install spatialgeometry spatialmath-python roboticstoolbox-python swift-sim qpsolvers pyyaml
pip3 install polyscope


# GTSAM Install 
git clone https://github.com/borglab/gtsam.git
cd ./gtsam
mkdir build
cd build
cmake ..
make check
make install
cd 

sudo snap install arduino

# Realsense
sudo apt install apt-transport-https
sudo mkdir -p /etc/apt/keyrings
curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp | sudo tee /etc/apt/keyrings/librealsense.pgp > /dev/null

echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] https://librealsense.intel.com/Debian/apt-repo `lsb_release -cs` main" | \
sudo tee /etc/apt/sources.list.d/librealsense.list
sudo apt-get update librealsense2-dkms librealsense2-utils librealsense2-dev librealsense2-dbg

sudo apt install librealsense2-dkms 


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

4. Initialize and update rosdep
```bash
sudo rosdep init
rosdep update
```


#### From here onwards, the steps apply to both personal computers

#### Setting up catkin workspace
The computers in MXLab have the above configuration. All the dependencies for this code to run have been installed. Please type the following commands in a terminal one after the other.
1. Source ROS commands
```bash
source /opt/ros/noetic/setup.bash
```
2. Create and initialize new catkin workspace. You may choose any name you like.
Here we chose **`mtrx5700space`** and it is located in the home directory.  
```bash
mkdir -p mtrx5700space/src && cd mtrx5700space
catkin init
```
