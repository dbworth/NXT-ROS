#!/usr/bin/env python

# Test the LEGO Mindstorms NXT motor

import roslib; roslib.load_manifest("nxt_python")
import rospy
import time # sleep()
import nxt.locator
from nxt.motor import *

def test_motor_cont(b):
    rospy.init_node('test_motor_cont')

    print "This is the motor controller test. \n"
    print "Make sure that you copied the MotorControl22.rxe firmware"
    print "to your NXT Brick and that a motor is plugged into Port A: \n"




    motor_A = Motor(b, PORT_A)

    # Start the Motor Controller program on the NXT Brick
    controller = nxt.motcont.MotCont(b)
    controller.start()

    controller.reset_tacho(nxt.PORT_A)

    power = 70
    goal = 360 # move 360 degrees
    speedreg = True # try to maintain speed under load
    smooth_start = False
    enable_brake = True # make controller hold goal position
    controller.cmd(nxt.PORT_A, power, goal, speedreg, smooth_start, enable_brake)

    while(1):
        # Get the motor position
        tacho_info = motor_A.get_tacho()
        print "position: ", tacho_info.rotation_count

        # Loop until is_ready=True
        finished = controller.is_ready(nxt.PORT_A)
        if finished:
            break;
    print "Holding position for 1 sec..."
    rospy.sleep(1.5)

    # Stop the controller, it will no
    # longer maintain the goal position
    controller.stop()


b = nxt.locator.find_one_brick()

test_motor_cont(b)

