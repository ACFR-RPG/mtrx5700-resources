# ROBOTIS Turtlebot 3

## On-board Compute
The Turtlebot 3's used in this course are equipped with a Raspberry Pi 3 Model B+. This does have enough compute for basic computer vision and data processing on-board. It is recommended that you do as much processing on-board the Turtlebot as possible to avoid latency over your network connection.


## WiFi Connection
Turtlebots used for the MTRX subjects at the University of Sydney now run custom software and have a specially designed hardware "hat" which allows the Turtlebots to reconfigure between an access point that broadcasts or WiFi connection to another network. We provide a document on Canvas used by MTRX3760 which outlines the configuation of the Turtlebots between these two modes.

Please note: students wishing to undertake projects with multiple Turtlebots will need to use a WiFi connection to another network, rather than individual Turtlebots (i.e. all Turtlebots must be in WiFi mode, on the same network - not AP mode).

## Testing Motors
The OpenCR board mounted above the motors on the Turtlebot3 has a number of buttons - holding each down for 3 seconds will cause the motors to actuate. If you are concerned one of your motors might be on the way out or you have issues with control, use one of these to confirm nominal operation of your Turtlebot motor controller. In doing so, this likely points the finger at your code and not the hardware.
