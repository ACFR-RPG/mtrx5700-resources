# ROBOTIS Turtlebot 3

ROBOTIS provides excellent documentation for an initial set-up of the Turtlebot 3. We direct your to their documentation [found here](https://emanual.robotis.com/docs/en/platform/turtlebot3/bringup/#bringup).

## On-board Compute
The Turtlebot 3's used in this course are equipped with a Raspberry Pi 3 Model B+. This does have enough compute for basic computer vision and data processing on-board. It is recommended that you do as much processing on-board the Turtlebot as possible to avoid latency over your network connection.


## WiFi Connection
Turtlebots used for the MTRX subjects at the University of Sydney now run custom software and have a specially designed hardware "hat" which allows the Turtlebots to reconfigure between an access point that broadcasts or WiFi connection to another network. We provide a document on Canvas used by MTRX3760 which outlines the configuation of the Turtlebots between these two modes.

Please note: students wishing to undertake projects with multiple Turtlebots will need to use a WiFi connection to another network, rather than individual Turtlebots (i.e. all Turtlebots must be in WiFi mode, on the same network e.g. KIRBYBOTS - not AP mode).

## Testing Motors
The OpenCR board mounted above the motors on the Turtlebot3 has a number of buttons - holding each down for 3 seconds will cause the motors to actuate. If you are concerned one of your motors might be on the way out or you have issues with control, use one of these to confirm nominal operation of your Turtlebot motor controller. In doing so, this likely points the finger at your code and not the hardware.


## Additional Sensors and Functionality
Since the Turtlebot3 has an on-board Raspberry Pi, you have access to the GPIO header. This is complete with UART, SPI, I2C and PWM lines which you may find useful for your major project.

Before discussing a general how to interface, we *caution* you. A Raspberry Pi runs off of 3.3V logic. Some sensors may need 5V logic. You therefore need a level-shifter, [such as this one](https://core-electronics.com.au/sparkfun-level-shifter-8-channel-txs01018e.html), which will convert the logic from 5V to 3.3V (and vice-versa). This is *imperative* as not obeying the GPIO logic voltage causes irreparable damage to the Raspberry Pi, and often renders it as needing to be binned. If you are not sure: don't plug it in.

Anything marketed as an Arduino sensor is fair game (watch the logic voltage) as is Raspberry Pi sensors. Keep in mind the power able to provided by the Raspberry Pi is limited so anything marketed as outside these purposes will need to be thoroughly checked. The Raspberry Pi has several GPIOs which can be reconfigured as additional UART lines, however for most sensor data the I2C bus or SPI lines are likely the best way of communicating. You may also find that adding a small microcontroller which handles some of your sensor processing, which then uses ROS-serial to send processed data may offload some compute from the Raspberry Pi (particularly if you are working with vision, and LiDAR, and an array of other sensors).

The Raspberry Pi also has a series of USB ports. This means that any number of machine vision cameras with a USB interface may also be installed onto the Turtlebot. An Intel Realsense for example may then be installed onto the Turtlebot to provide directional, dense depth information augmenting the sparse 360deg LiDAR information.
