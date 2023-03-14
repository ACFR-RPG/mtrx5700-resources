# ROBOTIS Turtlebot 3

ROBOTIS provides excellent documentation for an initial set-up of the Turtlebot 3. We direct your to their documentation [found here](https://emanual.robotis.com/docs/en/platform/turtlebot3/bringup/#bringup).

## On-board Compute
The Turtlebot 3's used in this course are equipped with a Raspberry Pi 3 Model B+. This does have enough compute for basic computer vision and data processing on-board. It is recommended that you do as much processing on-board the Turtlebot as possible to avoid latency over your network connection.


## WiFi Connection
Turtlebots used for the MTRX subjects at the University of Sydney now run custom software and have a specially designed hardware "hat" which allows the Turtlebots to reconfigure between an access point that broadcasts or WiFi connection to another network. We provide a document on Canvas developed by MTRX staff which outlines the configuation of the Turtlebots between these two modes.

Please note: students wishing to undertake projects with multiple Turtlebots will need to use a WiFi connection to another network, rather than individual Turtlebots (i.e. all Turtlebots must be in WiFi mode, on the same network e.g. KIRBYBOTS - not AP mode).

## Testing Motors
The OpenCR board mounted above the motors on the Turtlebot3 has a number of buttons - holding each down for 3 seconds will cause the motors to actuate. If you are concerned one of your motors might be on the way out or you have issues with control, use one of these to confirm nominal operation of your Turtlebot motor controller. In doing so, this likely points the finger at your code and not the hardware.


## Additional Sensors and Functionality
Since the Turtlebot3 has an on-board Raspberry Pi, you have access to the GPIO header. This is complete with UART, SPI, I2C and PWM lines which you may find useful for your major project.

Before discussing a general how to interface, we *caution* you. A Raspberry Pi runs off of 3.3V logic. Some sensors may need 5V logic. You therefore need a level-shifter, [such as this one](https://core-electronics.com.au/sparkfun-level-shifter-8-channel-txs01018e.html), which will convert the logic from 5V to 3.3V (and vice-versa). This is *imperative* as not obeying the GPIO logic voltage causes irreparable damage to the Raspberry Pi, and often renders it as needing to be binned. If you are not sure: don't plug it in.

Anything marketed as an Arduino sensor is fair game (watch the logic voltage) as is Raspberry Pi sensors. Keep in mind the power able to provided by the Raspberry Pi is limited so anything marketed as outside these purposes will need to be thoroughly checked. The Raspberry Pi has several GPIOs which can be reconfigured as additional UART lines, however for most sensor data the I2C bus or SPI lines are likely the best way of communicating. You may also find that adding a small microcontroller which handles some of your sensor processing, which then uses ROS-serial to send processed data may offload some compute from the Raspberry Pi (particularly if you are working with vision, and LiDAR, and an array of other sensors).

The Raspberry Pi also has a series of USB ports. This means that any number of machine vision cameras with a USB interface may also be installed onto the Turtlebot. An Intel Realsense for example may then be installed onto the Turtlebot to provide directional, dense depth information augmenting the sparse 360deg LiDAR information.

### Camera on Raspiberry Pi 3b + Ubuntu 20.04
We have installed a full ubuntu system onto the Raspberry Pi's in order to run Ubuntu 20.04 and ROS noetic. This means that, by default, the peripherals required by the camera are not set (and we do not have a raspi-config menu since this is a full ubuntu install).

VERY CAREFULLY modify the `/boot/firmware/config.txt` file and scroll down to the section `[all]`. The section should be modified to look like
```
[all]
arm_64bit=1
device_tree_address=0x03000000
start_x=1
gpu_mem=128
```
Save and reboot. 

Modifying the boot scripts should only be done if you know what you're doing and generally should be a last resort when solving a problem (however, this is not one of those situations). If you don't feel comfortable doing this, ask a tutor for help. 

ROS noetic does not support _raspi_cam_node_ that is standard in ROS melodic. Instead we use [cv camera](http://wiki.ros.org/cv_camera) package.
Install it via 
```
sudo apt install ros-noetic-cv-camera
```
It can be be run with rosrun, but we recommend making a launch file and adding it to the _turtlebot3_bringup_ packages to launch everything simultaneously. 
```
<launch>
  <node pkg="cv_camera" type="cv_camera_node" name="camera" output="screen">
    <param name="frame_id" value="camera_frame"/>
  </node>
</launch>
```
Launching the cv_camera node, you should see the topics
```
/camera/image_raw
/camera/camera_info
```
The streaming the camera's in raw over ROS is very slow; about 1-2Hz. ROS transport provides a set of image transport plugins which will automatically compress the stream for you. Install the plugins
```
sudo apt install ros-noetic-image-transport-plugins
```
Launching the cv_camera node after installing these plugins, compressed and theora topics should be registered with the rosmaster
```
/camera/image_raw
/camera/image_raw/compressed
/camera/image_raw/theora
```
The `/camera/image_raw/compressed` should stream to your local machine with minimal lag. NOTE: the compressed topic publishes `sensor_msgs/CompressedImage` messages, not `sensor_msgs/Image`. You will need to decode the data differently. 

The `cv_camera` node uses the [CameraInfoManager](http://wiki.ros.org/camera_info_manager) API to automatically publish the camera info topic from a configuration file. You will need to provide this file. If you are unfamilar with the camera info topic, it publishes `sensor_msgs/CameraInfo` messages which contain camera intrinsics, distortion, etc.. information. By default the package looks for the config file at __/home/ubuntu/.ros/camera_info/<frame_id>.yaml__, where the frame id is set via the param server. Note that you can change the file path by setting the `~camera_info_url` param.

The configuration yaml is the standard camera config yaml format, eg
```yaml
image_width: 1600
image_height: 1200
camera_name: narrow_stereo/left
camera_matrix:
  rows: 3
  cols: 3
  data: [2379.692159, 0, 810.664016, 0, 2364.724193, 565.047935, 0, 0, 1]
distortion_model: plumb_bob
distortion_coefficients:
  rows: 1
  cols: 5
  data: [-0.125793, 0.149249, -0.000565, -0.000972, 0]
rectification_matrix:
  rows: 3
  cols: 3
  data: [1, 0, 0, 0, 1, 0, 0, 0, 1]
projection_matrix:
  rows: 3
  cols: 4
  data: [2348.257291, 0, 809.568039, 0, 0, 2344.569053, 563.818346, 0, 0, 0, 1, 0]
```
NOTE: this is just an example set of values.
Your image width and height must match the actual size of the image and the camera_name should match the frame_id. You can verify that your config file has been loaded successfully by echoing the `/camera/camera_info` topic.
