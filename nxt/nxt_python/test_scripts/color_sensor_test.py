#!/usr/bin/env python

# Test the LEGO Mindstorms NXT 2.0 RGB color sensor

import roslib; roslib.load_manifest("nxt_python")
import rospy
import time
import nxt.locator
from nxt.sensor import *

def test_sensors(b):
        rospy.init_node('test_sensor')

        cs = Color20(b, PORT_1)
        print "This is the color sensor test make sure that your color sensor is plugged into port 1:"

	print 'RED:'
        cs.set_light_color(Type.COLORRED)
        time.sleep(1.0)

	print 'BLUE:'
        cs.set_light_color(Type.COLORBLUE)
        time.sleep(1.0)

	print 'GREEN:'
        cs.set_light_color(Type.COLORGREEN)
        time.sleep(1.0)

	print 'WHITE:'
        cs.set_light_color(Type.COLORFULL)
        time.sleep(1.0)

	print 'OFF:'
        cs.set_light_color(Type.COLORNONE)
        time.sleep(0.2)

        print "INTENSITY READING:"
        start = rospy.Time.now()
        while rospy.Time.now() < start+rospy.Duration(5.0):
          print 'INTENSITY:', cs.get_reflected_light(Type.COLORBLUE)
          time.sleep(0.1)

        print "COLOR READING:"
        start = rospy.Time.now()
        while rospy.Time.now() < start+rospy.Duration(5.0):
          print 'COLOR:', cs.get_color() # this uses Type.COLORFULL
          time.sleep(0.1)

        cs.set_light_color(Type.COLORNONE)


b = nxt.locator.find_one_brick()

test_sensors(b)

