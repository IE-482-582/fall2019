#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist

def gamepad_callback(msg, twist_pub):
	# Uncomment the next line to see what's being passed:
	# print msg

	t = Twist()
	t.angular.z = msg.angular.z
	t.linear.x  = msg.linear.x
	
	twist_pub.publish(t)

if __name__ == '__main__':
	# Initialize this ROS node:
	rospy.init_node('gamepad_to_twist')

	# Define our publisher (send to the turtlebot):
	twist_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

	# Subscribe to our joystick/gamepad:
	# "joy_twist" is the name of the topic
	# The topic is publishing messages of type "Twist"
	# When we hear from the topic, call the gamepad_callback callback.
	# Pass the definition of the twist publisher to the callback so it can publish messages.
	rospy.Subscriber('joy_twist', Twist, gamepad_callback, twist_pub)

	# Keep ROS from shutting down:
	rospy.spin()
	
