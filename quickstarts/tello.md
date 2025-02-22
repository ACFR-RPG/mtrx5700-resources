# Tello EDU Drones

IMPORTANT: Any students wishing to operate a drone in this course must acquire their remotely piloted aircraft (RPA) operator accreditation. This is free, and available online through the Civil Aviation Safety Authority (CASA). Information and an outline of how to obtain this may be found [at this link](https://www.casa.gov.au/drones/get-your-operator-credentials/operator-accreditation).

## ROS2
The Tello drones are not locked to ROS2 Humble as we use in this course.

Should you wish to use other versions of ROS or ROS2 you will need to source appropriate drivers (or write your own!) to operate the drones.

## Install and Setup
Refer to the installation instructions in [the Tello ROS2 driver repo for this course](https://github.com/ACFR-RPG/tello-driver-ros). 


## WiFi Connection
All Tello EDU Drones are their own WiFi hotspot. Connection to the drones is done via this link. Note you will need an ethernet connection to your laptop should you wish to retain internet connection during this time.

Each drone has their WiFi hotspot name printed in the battery bay and on a sticker on the top face of the drone. Make note of the drones unique hotspot name prior to inserting the battery and starting it up.

## Operation
Ensure the drone has at least a 5x5x5m volume to operate in, and one person is designated as a spotter for the drone, ensuring no one comes within the operating area. There should always therefore be at least 2 people with eyes on the drone at all times: the pilot and the spotter. 

When operating the drone be advised that you must only have one connection to the drone. Use of the Tello app to remote control the drone at the same time as an active ROS connection will cause the ROS connection to fail.


## Known Things to Watch Out For
If IMU data does not appear to change, you may need to power cycle the drone to refresh the IMU and it's topic publisher. If relying on IMU data where it feeds back into control of the drone through ROS, you *must* implement a sanity check of the IMU data before autonomous operation of the drone.

The video feed can be subject to some screen tearing and compression artefacts. Confirm you have a stable connection to the drone, line of sight between the controlling PC and the drone and that your code has checks for poor quality frames (which are denoted as having errors by the driver).

The drones go into sleep mode after some time of no operation, which may cause you to lose connection. You will need to restart the drone.


## Multiple Tello Drones
It is possible to configure the Tello EDU drones into a "station mode" which allows them to connect to the same WiFi network for swarm control. However, the caveat here is that the drones do not broadcast imagery in this mode and it is software locked.

A work around devised by the community comes in the form of using multiple connections and UDP forwarding to acquire streams from multiple drones. We have a number of Raspberry Pi's which are available for use in the Major Project stage of this course, and so those wishing to use multiple drones may find this a suitable configuration. Details found here: [https://github.com/clydemcqueen/udp_forward](https://github.com/clydemcqueen/udp_forward).
