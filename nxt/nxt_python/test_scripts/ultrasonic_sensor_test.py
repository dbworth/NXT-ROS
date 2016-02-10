#!/usr/bin/env python

# Test the LEGO Mindstorms NXT ultrasonic sensor

import roslib; roslib.load_manifest("nxt_python")
import rospy
import time
import nxt.locator
from nxt.sensor import *

def test_sensors(b):
    rospy.init_node('test_sensor')

    us = Ultrasonic(b, PORT_1)
    print "This is the ultrasonic sensor test. Make sure that your ultrasonic sensor is plugged into Port 1:"

    print "units", us.get_measurement_units() # 10e-2 meters

    us.command(us.Commands.CONTINUOUS_MEASUREMENT)
    start = rospy.Time.now()
    while rospy.Time.now() < start+rospy.Duration(5.0):
        print "CONTINUOUS MEASUREMENT DISTANCE:", us.get_distance()
        time.sleep(0.1)
        
    us.command(us.Commands.OFF) # Turn off the sensor
    print "\n"

    start = rospy.Time.now()
    while rospy.Time.now() < start+rospy.Duration(6.0):
        print "SENSOR WILL MEASURE SINGLE-SHOT DISTANCE IN 2 sec..."
        time.sleep(2.0)
        us.command(us.Commands.SINGLE_SHOT)
        print "DISTANCE:", us.get_distance()
        time.sleep(0.5)
        print "\n"

    us.command(us.Commands.OFF) # Turn off the sensor


b = nxt.locator.find_one_brick()

test_sensors(b)

