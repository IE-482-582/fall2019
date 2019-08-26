#!/usr/bin/env python

# listener2.py

import rospy
from std_msgs.msg import String
from custom_message.msg import grade_msg	

def grade_callback(data):
	'''
	string student_name
	float32 avg_score
	bool is_passing  
	'''	

	rospy.loginfo(rospy.get_caller_id() + " I heard: ")
	rospy.loginfo("\t Name: %s", data.student_name)
	rospy.loginfo("\t Avg Score: %f", data.avg_score)
	rospy.loginfo("\t Passing? %s", data.is_passing)
    
def listener():

    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("this_msg", grade_msg, grade_callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
