NXT-ROS
=======

A software stack containing tools for using LEGO Mindstorms NXT with ROS. 

This branch depends on:<br> 	&#8226; ROS Fuerte<br> &#8226; Python 2.6+<br> &#8226; NXT-Python v2.2.0

Forked from the version for ROS Electric: http://stack-nxt.foote-ros-pkg.googlecode.com/hg  
Original documentation: [wiki.ros.org/nxt](http://wiki.ros.org/nxt)  

.

###Package status

All packages use `rosbuild`.

**nxt** stack:  

 - **nxt_controllers**  
Compiles ok, the base_controller has been tested.

 - **nxt_description**  
Working.

 - **nxt_lxf2urdf**  
Compiles ok, un-tested.

 - **nxt_msgs**  
Working.

 - **nxt_python**  
Working.

 - **nxt_ros**  
Working.  
There is a new option to read the position of a motor as the absolute angle from 0 to 2*pi radians. By default the motor position is measured as the total +/- rotation in either direction, relative to the starting position.

 - **nxt_rviz_plugin**  
Disabled, it is not compatible with the ROS Fuerte release of RViz which was ported to Qt.

**nxt_robots** stack:  

 - **nxt_robot_gyro_car**  
Working. The robot model can be viewed in Rviz.  

 - **nxt_robot_kit_test**  
Working. This is a test package, so the robot looks like a pile of disconnected LEGO parts when viewed in Rviz.  

 - **nxt_robot_sensor_car**  
Working. The robot model can be viewed in Rviz.  

**nxt_apps** stack:  

 - **nxt_assisted_teleop**  
Compiles ok, un-tested.

 - **nxt_teleop**  
Working.  

.

###Documentation  
Installation instructions are in this README file.  

The original documentation has more detailed information:  
[wiki.ros.org/nxt](http://wiki.ros.org/nxt)  
[wiki.ros.org/nxt_robots](http://wiki.ros.org/nxt_robots)  
[wiki.ros.org/nxt_apps](http://wiki.ros.org/nxt_apps)  

.

###Installation

Upgrade the firmware on your NXT brick.  
The minimum required version is 1.28, but 1.31 is the latest as of Jan 2016.

Clone the Fuerte branch of this repository:  
> $ cd ~/  
> $ git clone https://github.com/dbworth/NXT-ROS.git --branch fuerte

Copy the 3 stacks into your ROS Workspace.  
Note: you only need the `nxt` stack to get started.
> $ cp NXT-ROS/nxt ~/ROS_WORKSPACE/  
> $ cp NXT-ROS/nxt_robots ~/ROS_WORKSPACE/  
> $ cp NXT-ROS/nxt_apps ~/ROS_WORKSPACE/  

Add the stacks to your $ROS_PACKAGE_PATH.  
You might like to add these commands to your `~/.bashrc` file.  
> $ export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/ros_workspace/nxt  
> $ export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/ros_workspace/nxt_robots  
> $ export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/ros_workspace/nxt_apps  

**Install dependencies:**  

NXT-Python v1.2.0 is automatically installed in the `/src` directory of the `nxt_python` package when we run `rosmake`.

You can install the other dependencies using the Ubuntu Package Manager or ROSdep: 
> $ sudo apt-get install ros-fuerte-ros-comm ros-fuerte-common-msgs ros-fuerte-joystick-drivers ros-fuerte-visualization ros-fuerte-navigation  
> $ sudo aptitude install libusb-dev python-usb python-bluez  

or 

> $ sudo rosdep install -y nxt  
> $ sudo rosdep install -y nxt_robots  
> $ sudo rosdep install -y nxt_apps  

Compile the stacks of packages:  
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

For Ubuntu 12.04 use:  
> $ echo "SUBSYSTEM==\"usb\", ATTRS{idVendor}==\"0694\", GROUP=\"lego\", MODE=\"0660\"" > /tmp/70-lego.rules && sudo mv /tmp/70-lego.rules /etc/udev/rules.d/70-lego.rules

Restart udev:
> $ sudo restart udev

Log-out or restart your computer.


###Test the installation:  

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
Connect the NXT to your computer.  

> $ roscore

In a new terminal:  
> $ roslaunch nxt_robot_gyro_car robot.launch  

Ignore the error messages, this is because we haven't connected the required motors and sensors.  

In a new terminal:  
> $ rosrun rviz rviz  

In Rviz you'll need to set the Fixed Frame to '/base_link' and add a RobotModel display so you can visualize the robot.

**nxt_apps** stack:  

In a new terminal:  
> $ roslaunch nxt_teleop teleop_keyboard.launch  

Use the arrow keys to control the motors.  

You can also use a joystick.  
Configure the joystick using the instructions here:  
http://wiki.ros.org/joy/Tutorials/ConfiguringALinuxJoystick

> $ roslaunch nxt_teleop teleop_joy.launch 

With the Microsoft XBox 360 joystick, you need to hold down the top-left button. The left joystick controls turning, the right joystick goes forward.  

=============





