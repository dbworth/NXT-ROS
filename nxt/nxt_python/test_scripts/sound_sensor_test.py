#!/usr/bin/env python

# Test the LEGO Mindstorms NXT sound sensor

import roslib; roslib.load_manifest("nxt_python")
import rospy
import time
import nxt.locator
from nxt.sensor import *

def test_sensors(b):
    rospy.init_node('test_sensor')

    ss = Sound(b, PORT_1)
    print "This is the sound sensor test. Make sure that your sound sensor is plugged into Port 1:"

    ss.set_adjusted(active=False) # un-adjusted
    start = rospy.Time.now()
    while rospy.Time.now() < start+rospy.Duration(5.0):
        print 'UN-ADJUSTED SOUND READING:', ss.get_loudness() # get the scaled value
        time.sleep(0.1)

    ss.set_adjusted(active=True) # adjusted
    start = rospy.Time.now()
    while rospy.Time.now() < start+rospy.Duration(5.0):
        print 'ADJUSTED SOUND READING:', ss.get_loudness() # get the scaled value
        time.sleep(0.1)


b = nxt.locator.find_one_brick()

test_sensors(b)

