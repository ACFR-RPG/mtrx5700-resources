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


# ROS2 Humble
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update

sudo apt install ros-humble-desktop-full
sudo apt-get install ros-humble-ros2-control ros-humble-ros2-controllers ros-humble-clearpath-ros2-socketcan-interface ros-humble-moveit ros-humble-moveit-visual-tools ros-humble-moveit-ros-control-interface  ros-humble-moveit-resources ros-humble-derived-object-msgs ros-humble-gazebo-ros ros-humble-gazebo-ros2-control ros-humble-tf2-eigen ros-humble-actionlib-msgs ros-humble-control-msgs ros-humble-controller-interface ros-humble-controller-manager ros-humble-effort-controllers ros-humble-geometry-msgs ros-humble-hardware-interface ros-humble-ur-msgs ros-humble-ur-client-library ros-humble-ur-dashboard-msgs ros-humble-ur-description ros-humble-kinematics-interface-kdl python3-pymodbus ros-humble-diagnostic-updater


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

# Tello Drones
sudo apt install ros-humble-as2-platform-tello ros-humble-teleop-twist-keyboard
mkdir -p ~/ros2_mtrx5700ws/src && cd ~/ros2_mtrx5700ws/src
git clone https://github.com/tentone/tello-ros2.git # requires OpenCV 3 to install SLAM. Can remove SLAM folder before building if not needed

# Turtlebot 3
sudo apt-get install ros-humble-joy ros-humble-teleop-twist-joy ros-humble-teleop-twist-keyboard ros-humble-laser-proc ros-humble-nav2-amcl ros-humble-nav2-map-server ros-humble-urdf ros-humble-xacro ros-humble-compressed-image-transport ros-humble-rqt* ros-humble-rviz2 ros-humble-navigation2 ros-humble-interactive-markers ros-humble-dynamixel-sdk ros-humble-turtlebot3*

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
sudo make install

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
On your personal computer and MXLab account, you can automate the command to run in each new bash terminal:
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
