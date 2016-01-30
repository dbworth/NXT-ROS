NXT-ROS
=======

A software stack containing tools to interface LEGO Mindstorms with ROS Fuerte.

Forked from the version for ROS Electric: http://stack-nxt.foote-ros-pkg.googlecode.com/hg  
Original documentation: [wiki.ros.org/nxt](http://wiki.ros.org/nxt)  


###Installation

Upgrade the firmware on your NXT brick. The minimum required version is 1.28, but 1.31 is the latest as of Jan 2016.

Clone the Fuerte branch of this repository:  
> $ cd ~/
> $ git clone https://github.com/dbworth/NXT-ROS.git --branch fuerte

Copy the 3 stacks into your ROS Workspace.  
Note: you only need the `nxt` stack to get started.
> $ cp NXT-ROS/nxt ~/ROS_WORKSPACE/  
> $ cp NXT-ROS/nxt-apps ~/ROS_WORKSPACE/  
> $ cp NXT-ROS/nxt-robots ~/ROS_WORKSPACE/  

Add the stacks to your $ROS_PACKAGE_PATH.  
You might like to add these commands to your `~/.bashrc~ file.  
> $ export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/ros_workspace/nxt  
> $ export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/ros_workspace/nxt-apps  
> $ export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/ros_workspace/nxt-robots  

Install dependencies:  
Using ROSdep:  
> $ sudo rosdep install -y nxt  
> $ sudo rosdep install -y nxt-apps  
> $ sudo rosdep install -y nxt-robots  

or using Ubuntu Package Manager:    
> $ sudo aptitude install libusb-dev python-usb python-bluez  

Compile the stacks of packages:  
> $ rosmake nxt

###Set the USB permissions:

Add a new group:
> $ sudo groupadd lego

Add yourself to that group:
> $ sudo usermod -a -G lego USERNAME

Create a udev rules file for the lego group that you just created.
> $ echo "BUS==\"usb\", ATTRS{idVendor}==\"0694\", GROUP=\"lego\", MODE=\"0660\"" > /tmp/70-lego.rules && sudo mv /tmp/70-lego.rules /etc/udev/rules.d/70-lego.rules

On ubuntu 12.04 use
> $ echo "SUBSYSTEM==\"usb\", ATTRS{idVendor}==\"0694\", GROUP=\"lego\", MODE=\"0660\"" > /tmp/70-lego.rules && sudo mv /tmp/70-lego.rules /etc/udev/rules.d/70-lego.rules

Restart udev:
> $ sudo restart udev

Log-out or restart your computer.


###Test the installation:
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
