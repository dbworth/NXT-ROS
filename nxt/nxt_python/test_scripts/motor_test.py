#!/usr/bin/env python

# Test the LEGO Mindstorms NXT motor

import roslib; roslib.load_manifest("nxt_python")
import rospy
import time # sleep()
import nxt.locator
from nxt.motor import *

def test_motor(b):
    rospy.init_node('test_motor')

    print "This is the motor test. Make sure that a motor is plugged into Port A:"
    motor_A = Motor(b, PORT_A)

    print "Turning motor forwards"
    power = 100 # value between -128 and 127
    distance = 360 # turn 360 degrees
    motor_A.turn(power, distance)

    print "Turning motor in reverse"
    power = -100
    distance = 360
    motor_A.turn(power, distance)

    print "Running motor continuously for 3 sec"
    power = 100
    regulated = False
    motor_A.run(power, regulated)
    start = rospy.Time.now()
    while rospy.Time.now() < start+rospy.Duration(3.0):

        # Get info about the internal state, and
        # the rotational state of the motor
        (output_state, tacho_state) = motor_A._read_state()
        print "output_state: ", output_state
        print "tach_info: ", tacho_state # tacho_count, block_tacho_count, rotation_count

        # Get just the motor position
        tacho_info = motor_A.get_tacho()
        print "position: ", tacho_info.rotation_count
        time.sleep(0.1)
    print '\n'

    motor_A.run(0) # stop motor

b = nxt.locator.find_one_brick()

test_motor(b)

