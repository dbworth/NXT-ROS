#!/usr/bin/env python

# Test the LEGO Mindstorms NXT motor

import roslib; roslib.load_manifest("nxt_python")
import rospy
import time # sleep()
import sys # stdout.write()
import nxt.locator
from nxt.motor import *

def test_motor(b):
    rospy.init_node('test_motor')

    print "This is the motor test. Make sure that a motor is plugged into Port A:"
    motor = Motor(b, PORT_A)

    print "Turning motor forwards"
    power = 100 # value between -128 and 127
    distance = 360 # turn 360 degrees
    motor.turn(power, distance)

    print "Turning motor in reverse"
    power = -100
    distance = 360
    motor.turn(power, distance)

    print "Running motor continuously for 3 sec"
    power = 100
    regulated = False
    motor.run(power, regulated)
    start = rospy.Time.now()
    while rospy.Time.now() < start+rospy.Duration(3.0):
        sys.stdout.write('.')
        sys.stdout.flush()
        time.sleep(0.1)
    print '\n'
    motor.run(0) # stop motor


b = nxt.locator.find_one_brick()

test_motor(b)

