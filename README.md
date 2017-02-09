NXT-ROS
=======

A software stack containing tools for using LEGO Mindstorms NXT with ROS.

This branch depends on:<br> 	&#8226; ROS Kinetic<br> &#8226; Python 2.x<br> &#8226; NXT-Python v2.2.2  

Originally forked from: http://stack-nxt.foote-ros-pkg.googlecode.com/hg  

.

###Documentation  
Installation instructions are in this README file.  

The original documentation has more detailed information:  
[wiki.ros.org/nxt](http://wiki.ros.org/nxt)  
[wiki.ros.org/nxt_robots](http://wiki.ros.org/nxt_robots)  
[wiki.ros.org/nxt_apps](http://wiki.ros.org/nxt_apps)  
.

###Package status

These packages use `rosbuild` and were tested  
with ROS Kinetic on Ubuntu 16.04.

The following features have been tested:

**nxt** stack:  

 - **nxt_controllers**  
 - **nxt_description**  
 - **nxt_lxf2urdf**  
 - **nxt_msgs**  
 - **nxt_python**  
 - **nxt_ros**  
 - **nxt_ros_test**  
 - **nxt_rviz_plugin**  
**nxt_robots** stack:  
 - **nxt_robot_gyro_car**  
 - **nxt_robot_kit_test**  
 - **nxt_robot_sensor_car**  
**nxt_apps** stack:  
 - **nxt_assisted_teleop**  
 - **nxt_teleop**  

.


###Installation

Upgrade the firmware on your NXT brick.  
The minimum required version is 1.28, but 1.31 is the latest as of Jan 2016.

Clone the Kinetic branch of this repository:  
> $ cd ~/  
> $ git clone https://github.com/dbworth/NXT-ROS.git --branch kinetic

Copy the 3 stacks into your ROS package path.  
Note: you only need the `nxt` stack to get started.
> $ cp NXT-ROS/nxt $ROS_PACKAGE_PATH  
> $ cp NXT-ROS/nxt_robots $ROS_PACKAGE_PATH  
> $ cp NXT-ROS/nxt_apps $ROS_PACKAGE_PATH   

**Install dependencies:**  

NXT-Python v2.2.2 is automatically installed in the `/src` directory of the `nxt_python` package when we run `rosmake`.

You can install the other dependencies using the Ubuntu Package Manager or ROSdep:
> $ sudo apt-get install ogre-1.9-tools ros-kinetic-bfl libcppunit-dev libcppunit-1.13-0v5 ros-kinetic-joy ros-kinetic-robot-pose-ekf ros-kinetic-costmap-2d ros-kinetic-navigation
> $ sudo apt-get install libbluetooth-dev
> $ sudo pip install PyBluez PyUSB

Compile the stacks of packages:  
> $ rosmake nxt  

Than you will get an *error* while compilling *linest_rviz_plugin*.  
Thats because of parse error at **"BOOST_JOIN"**.  
This macros are used to define a namespace name.  
I don't know what the right way to fix this but I know two ways to solve it:  

1. Add to file, where including file with **"BOOST_JOIN"** inside (like **has_binary_operator.hp**)  

   Example:
   ```
   #ifndef Q_MOC_RUN
   #include <boost/type_traits/detail/has_binary_operator.hp>
   #endif
   ```

   But it didn't help me. Mb I used it wrong way.  

2. Compile **nxt** by parts.

   Compile first part of **nxt** with last **libboost**, then downgrade to **1.48** and continue compiling  and then go back to last version and end compiling.
   > $ sudo dpkg -P --force-all libboost-all-dev  
   > $ sudo dpkg -P --force-all libboost1.58-dev:amd64  
   > $ sudo dpkg -P --force-all libboost1.58-tools-dev  

   > $ rosmake nxt  

   > $ sudo apt-get update  
   > $ sudo apt-get -f install  

   > $ rosmake nxt  


> $ rosmake nxt_robots  
> $ rosmake nxt_apps  

**Set the USB permissions:**

Add a new group:
> $ sudo groupadd lego

Add yourself to that group:
> $ sudo usermod -a -G lego USERNAME

For Ubuntu versions before 12.0, create a udev rules file for the lego group that you just created:  
> $ echo "BUS==\"usb\", ATTRS{idVendor}==\"0694\", GROUP=\"lego\", MODE=\"0660\"" > /tmp/70-lego.rules && sudo mv /tmp/70-lego.rules /etc/udev/rules.d/70-lego.rules

Restart udev:
> $ sudo system

Log-out or restart your computer.


###Test the installation:  

Connect your NXT Brick to the computer via USB cable.  
You can run this utility to display information about your Brick:  
> $ rosrun nxt_python nxt_test

You should see the following output:  
```
NXT brick name: MyBrick
Host address: 00:16:00:00:D3:00
Bluetooth signal strength: 0
Free user flash: 25872
Protocol version 1.124
Firmware version 1.31
Battery level 7602 mV
```

**nxt** stack:  

Connect a touch sensor to port 1.  
> $ roscore  
> $ rosrun nxt_python touch_sensor_test.py  

When you push the touch sensor, the output on the screen will change.  
The node automatically exits are 5 seconds.

Now connect a motor to port A.  
> $ roslaunch nxt_ros_test test.launch

Check the status of the touch sensor using ROS:  
> $ rostopic echo /my_touch_sensor  

When you push the touch sensor, the boolean value published in the ROS msg will change.

Read the position of the motor using ROS:  
>  $ rostopic echo /joint_states  

When you rotate the motor shaft by hand, the position and velocity values published in the ROS msg will change.

Make the motor rotate using ROS:  
> $ rostopic pub /joint_command nxt_msgs/JointCommand '{name: 'motor_joint', effort: 0.8}' --once  

The 'effort' value can be in the range 0.0 to 1.25, with +/- to specify the direction.

To stop the motor, set the effort to 0.0:  
> $ rostopic pub /joint_command nxt_msgs/JointCommand '{name: 'motor_joint', effort: 0.0}' --once  

**nxt_robots** stack:  

Connect motors to Port A and Port B of the NXT Brick.  
Connect an ultrasonic sensor to Port 2.  
Connect the NXT to your computer.  

> $ roscore

In a new terminal:  
> $ roslaunch nxt_robot_gyro_car robot.launch  

In a new terminal:  
> $ rosrun rviz rviz  

In Rviz you'll need to set the Fixed Frame to '/base_link'.  
Add a RobotModel display so you can visualize the robot.

**nxt_apps** stack:  

In a new terminal:  
> $ roslaunch nxt_teleop teleop_keyboard.launch  

Use the arrow keys to control the motors.  

You can also use a joystick.  
Configure the joystick using the instructions here:  
http://wiki.ros.org/joy/Tutorials/ConfiguringALinuxJoystick

> $ roslaunch nxt_teleop teleop_joy.launch

With the Microsoft XBox 360 joystick, you need to hold down the top-left button. The left joystick controls turning, the right joystick goes forward.  

**MotorControl** firmware:  

[MotorControl](http://www.mindstorms.rwth-aachen.de/trac/wiki/MotorControl) is a program that can be optionally installed on your NXT Brick to provide more accurate control of the motors using python. Instead of sending commands directly to the motor, you send commands to the onboard controller which tracks the status of the motors.

Copy the MotorControl program to your NXT Brick.  
You must first change to the directory containing the *.rxe file:  
> $ roscd nxt_python/src/MotorControl/  

Connect to your Brick via USB cable and transfer the program binary:

> $ rosrun nxt_python nxt_push MotorControl22.rxe

Test the controller using the provided test script:
> $ roscore  
> $ rosrun nxt_python motor_cont_test.py  


=============
