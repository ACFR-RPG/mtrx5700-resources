# Universal Robots UR5e Robotic Arm (ROS2)

## Start-up and Operation
Before powering the UR5e ensure the arm is at least 1m from any walls, tables or solid objects. Clear the work area - in particular ensure no laptops are on the table. A UR5e Vs. a laptop screen has a clear winner.

Ensure there is one person in control of the teach pendant, with CLEAR SIGHT of the robotic arm, at ALL times. Should the UR5e move unexpectedly as part of your autonomous control of the robot, you want to be able to slow down its motion or emergency stop it to avoid any damage to the robot or the lab.

Turn on the UR5e using the power button on the teach pendant. This powers on the computer, but does not power the robot itself. In the bottom right of the screen once the computer has booted, click to power on and enable the robot. Ensure you are clear of the robot during this time - should there be a mismatch between where the robot thinks it is - it is possible that the arm will move to correct this.

Once the robot is powered on, it is good practice to manually freedrive the arm (using the button on the back of the teach pendant) to a configuration close to where the working configuration of the arm will be. You should also check the angles of the UR5e, particularly the wrist, and freedrive any to the middle of their axes to avoid wind-up of joints to the end of their physical limits. Good practice is also to use the teach pendant to test movement through all axes and ensure the arm is working nominally.


## Documentation
Please note that you should have ros2 humble installed if you are following the environment setup instruction [Ubuntu 22.04 Environment (2025)](../environment/22_04_ros2_humble.md). See the [installation instructions](https://docs.universal-robots.com/Universal_Robots_ROS2_Documentation/doc/ur_robot_driver/ur_robot_driver/doc/installation/installation.html) for more details and source-build instructions, and you are encouraged to read more from this official documentation instead of this "quick start note".


## Dependencies
Git clone the following repositorties into your workspace ("$COLCON_WS/src")
 - https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver
 - https://github.com/UniversalRobots/Universal_Robots_Client_Library

Note that instead of catkin, ROS2 is using a colcon workspace. Clone relevant packages, install dependencies, compile, and source the workspace by using:
```
cd $COLCON_WS
git clone -b humble https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver.git src/Universal_Robots_ROS2_Driver
vcs import src --skip-existing --input src/Universal_Robots_ROS2_Driver/Universal_Robots_ROS2_Driver-not-released.${ROS_DISTRO}.repos
rosdep update
rosdep install --ignore-src --from-paths src -y
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

When consecutive pulls lead to build errors it is possible that you’ll have to build an upstream package from source, as well. Install the missing upstream package by: 

```
cd $COLCON_WS
vcs import src --skip-existing --input src/Universal_Robots_ROS2_Driver/Universal_Robots_ROS2_Driver.${ROS_DISTRO}.repos
rosdep update
rosdep install --ignore-src --from-paths src -y
```

## ROS2 Connection
Onboard the teach pendant there is an `external control` script which needs to be run to enable communcation between the UR5e and ROS. Run this script prior to running the below commands.

```bash
# Check the IP and whether calibration for the UR5e is available (ask your tutor).
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=<UR_TYPE> robot_ip:=<IP_OF_THE_ROBOT> launch_rviz:=true
```

To use mocked hardware (a ros2 simulator, capability of ros2_control), use use_mock_hardware argument, like:
```bash
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e robot_ip:=yyy.yyy.yyy.yyy use_mock_hardware:=true
```

For assignments, you may need to run slightly different commands to run the robot. Where this is the case, we will document these inside the appropriate assignment folder.

## Robotiq Hand-E Gripper
The UR5e in the teaching lab is equipped with a Robotiq Hand-E gripper. Actuation of this gripper is achieved over a USB connection. Despite the cable from the gripper going into the main UR5e computer, the USB cable must still be inserted into either the teach pendant or the controlling computer to communicate.

Documentation and software for the gripper is provided through the [Robotiq website](https://robotiq.com/support/hand-e-adaptive-robot-gripper). If you need to update the URCap, please seek assistance from a tutor.

<!-- To run the gripper, you will need to execute the following (all in separate terminals).
```bash
# This creates the ROS node for the gripper.
rosrun robotiq_2f_gripper_control Robotiq2FGripperRtuNode.py /dev/ttyUSB0

# If the above fails, you will need to update permissions of the USB connection. 
# Note you will need a tutor to run this command from the lab PCs
sudo chmod 777 /dev/ttyUSB0
```
The above creates a ROS node, you may then link the hardware to your Gazebo simulation.
```bash
# Execute the below, then type 'r' enter, 'a' enter.
rosrun robotiq_2f_gripper_control Robotiq2FGripperSimpleController.py

# This links to Gazebo.
roslaunch gazebo_ros_link_attacher test_attacher.launch
``` -->

## Adding Cameras or Attachments
As part of this course, you will likely want to equip the UR5e with a vision system, some extra sensing capability, human interfaces etc. This is both expected and encouraged!

You may wish to remove the gripper and create interfaces on the tool flange. You may wish to attach something to the wrist of the UR5e. Whilst there are some existing mounts for Intel Realsenses, we encourage you to do some design yourself to create appropriate hardware.

You may find the following resources useful in creating these new mounts:

[UR5e user manual](https://s3-eu-west-1.amazonaws.com/ur-support-site/40971/UR5e_User_Manual_en_Global.pdf)

[UR5e Technical Specifications](https://www.universal-robots.com/media/1807465/ur5e-rgb-fact-sheet-landscape-a4.pdf)

Please consult the tutors in how you may include these in simulation for collision detection, interfacing etc.
