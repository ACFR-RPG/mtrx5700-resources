# Universal Robots UR5e Robotic Arm

## Start-up and Operation
Before powering the UR5e ensure the arm is at least 1m from any walls, tables or solid objects. Clear the work area - in particular ensure no laptops are on the table. A UR5e Vs. a laptop screen has a clear winner.

Ensure there is one person in control of the teach pendant at all times. Should the UR5e move unexpectedly as part of your autonomous control of the robot, you want to be able to slow down its motion or emergency stop it to avoid any damage to the robot or the lab.

Turn on the UR5e using the power button on the teach pendant. This powers on the computer, but does not power the robot itself. In the bottom right of the screen once the computer has booted, click to power on and enable the robot. Ensure you are clear of the robot during this time - should there be a mismatch between where the robot thinks it is - it is possible that the arm will move to correct this.

Once the robot is powered on, it is good practice to manually freedrive the arm (using the button on the back of the teach pendant) to a configuration close to where the working configuration of the arm will be. You should also check the angles of the UR5e, particularly the wrist, and freedrive any to the middle of their axes to avoid wind-up of joints to the end of their physical limits. Good practice is also to use the teach pendant to test movement through all axes and ensure the arm is working nominally.


## ROS Connection
Onboard the teach pendant there is an `external control` script which needs to be run to enable communcation between the UR5e and ROS. Run this script prior to running the below commands.

```bash
# Check the IP and whether calibration for the UR5e is available (ask your tutor).
roslaunch ur_robot_driver ur5e_bringup.launch robot_ip:=172.17.XX.XX limited:=true kinematics_config:=/path/to/kinematics/mxlab_calib.yaml

# Note here the sim:=false flag.
roslaunch ur5_e_moveit_config ur5_e_moveit_planning_execution.launch sim:=false

# RViz to launch.
roslaunch ur5_e_moveit_config moveit_rviz.launch config:=true
```


## Robotiq Hand-E Gripper
The UR5e in the teaching lab is equipped with a Robotiq Hand-E gripper. Actuation of this gripper is achieved over a USB connection. Despite the cable from the gripper going into the main UR5e computer, the USB cable must still be inserted into either the teach pendant or the controlling computer to communicate.

Documentation and software for the gripper is provided through the [Robotiq website](https://robotiq.com/support/hand-e-adaptive-robot-gripper). If you need to update the URCap, please seek assistance from a tutor.

To run the gripper, you will need to execute the following (all in separate terminals).
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
```

## Adding Cameras or Attachments
As part of this course, you will likely want to equip the UR5e with a vision system, some extra sensing capability, human interfaces etc. This is both expected and encouraged!

You may wish to remove the gripper and create interfaces on the tool flange. You may wish to attach something to the wrist of the UR5e. Whilst there are some existing mounts for Intel Realsenses, we encourage you to do some design yourself to create appropriate hardware.

You may find the following resources useful in creating these new mounts:
[UR5e user manual](https://s3-eu-west-1.amazonaws.com/ur-support-site/40971/UR5e_User_Manual_en_Global.pdf)
[UR5e Technical Specifications](https://www.universal-robots.com/media/1807465/ur5e-rgb-fact-sheet-landscape-a4.pdf)


## Dependencies
 - git@github.com:pal-robotics/gazebo_ros_link_attacher.git
 - https://github.com/UniversalRobots/Universal_Robots_ROS_Driver
 - https://github.com/UniversalRobots/Universal_Robots_Client_Library
 - https://github.com/ros-industrial/universal_robot
 - https://github.com/ros-industrial/robotiq

```
sudo apt update -qq && rosdep update && rosdep install --from-paths src --ignore-src -y
catkin build
```
