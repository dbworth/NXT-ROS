#!/usr/bin/env python

#
# Script to publish dummy msgs
# to test the NXT Rviz plugins.
#

import roslib; roslib.load_manifest('nxt_rviz_plugin')
from nxt_msgs.msg import Color
from nxt_msgs.msg import Range
import rospy
import tf

color_topic = '/my_color_sensor'
color_publisher = rospy.Publisher(color_topic, Color)
color_values = list()
color_values.append([0.0,0.0,0.0]) # black
color_values.append([0.0,0.0,1.0]) # blue
color_values.append([0.0,1.0,0.0]) # green
color_values.append([1.0,1.0,0.0]) # yellow
color_values.append([1.0,0.0,0.0]) # red
color_values.append([1.0,1.0,1.0]) # white
color_count = 0

range_topic = '/my_range_sensor'
range_publisher = rospy.Publisher(range_topic, Range)
range_values = list()
range_values.append(0.05)
range_values.append(0.07)
range_values.append(0.09)
range_values.append(0.11)
range_values.append(0.13)
range_count = 0

rospy.init_node('send_test_msgs')

print "Publishing test Color messages to topic ", color_topic
print "Publishing test Range messages to topic ", range_topic

br = tf.TransformBroadcaster()

rate = rospy.Rate(1)
while not rospy.is_shutdown():

    color_msg = Color()
    color_msg.header.frame_id = "/color_sensor_frame"
    color_msg.header.stamp = rospy.Time.now()
    color_msg.r = color_values[color_count][0]
    color_msg.g = color_values[color_count][1]
    color_msg.b = color_values[color_count][2]
    color_publisher.publish(color_msg)
    color_count = color_count + 1
    if color_count > (len(color_values)-1):
        color_count = 0

    range_msg = Range()
    range_msg.header.frame_id = "/range_sensor_frame"
    range_msg.header.stamp = rospy.Time.now()
    range_msg.range = range_values[range_count]
    range_msg.spread_angle = 0.2
    range_publisher.publish(range_msg)
    range_count = range_count + 1
    if range_count > (len(range_values)-1):
        range_count = 0

    br.sendTransform((0, 0, 0),
                     tf.transformations.quaternion_from_euler(0, 0, 0),
                     rospy.Time.now(),
                     "color_sensor_frame",
                     "map")

    br.sendTransform((0, 0, 0),
                     tf.transformations.quaternion_from_euler(0, 0, 0),
                     rospy.Time.now(),
                     "range_sensor_frame",
                     "map")

    rate.sleep()

