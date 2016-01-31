#!/usr/bin/env python

# Test the LEGO Mindstorms NXT light sensor

import roslib; roslib.load_manifest("nxt_python")
import rospy
import time
import nxt.locator
from nxt.sensor import *

def test_sensors(b):
        rospy.init_node('test_sensor')

        ls = Light(b, PORT_1)
        print "This is the light sensor test. Make sure that your light sensor is plugged into Port 1:"

        ls.set_illuminated(active=False)
        start = rospy.Time.now()
        while rospy.Time.now() < start+rospy.Duration(5.0):
          print 'AMBIENT LIGHT READING:', ls.get_lightness() # get the scaled value
          time.sleep(0.1)

        ls.set_illuminated(active=True)
        start = rospy.Time.now()
        while rospy.Time.now() < start+rospy.Duration(5.0):
          print 'ACTIVE LIGHT READING:', ls.get_lightness() # get the scaled value
          time.sleep(0.1)

        # Turn off the light
        ls.set_illuminated(active=False)


b = nxt.locator.find_one_brick()

test_sensors(b)

