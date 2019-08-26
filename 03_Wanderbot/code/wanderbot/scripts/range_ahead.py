#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan

def scan_callback(msg):
	print len(msg.ranges)
	print msg.angle_min
	print msg.angle_max
	print msg.angle_increment
	print (msg.angle_max - msg.angle_min) / msg.angle_increment

	range_ahead = msg.ranges[len(msg.ranges)/2]
	print "range ahead: %0.1f" % range_ahead

rospy.init_node('range_ahead')

scan_sub = rospy.Subscriber('scan', LaserScan, scan_callback)
rospy.spin()


