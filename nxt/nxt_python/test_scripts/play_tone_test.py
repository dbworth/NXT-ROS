#!/usr/bin/env python

# Test the LEGO Mindstorms NXT on-board speaker

import roslib; roslib.load_manifest("nxt_python")
import rospy
import nxt.locator

FREQ_C = 523
FREQ_D = 587
FREQ_E = 659
FREQ_G = 784

rospy.init_node('test_play_tone')

print "This is the on-board speaker test."
print "Make sure that the speaker volume on your NXT Brick is set above zero."

b = nxt.locator.find_one_brick()

b.play_tone_and_wait(FREQ_C, 500)
b.play_tone_and_wait(FREQ_D, 500)
b.play_tone_and_wait(FREQ_E, 500)
b.play_tone_and_wait(FREQ_G, 500)

