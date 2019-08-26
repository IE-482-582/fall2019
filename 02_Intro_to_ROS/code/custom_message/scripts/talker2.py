#!/usr/bin/env python

# talker2.py

import rospy
from std_msgs.msg import String
from custom_message.msg import grade_msg	
# custom_message.msg:
#	custom_message is the name of our package.
# 	We have a directory in that package called "msg".
# grade_msg:
#	Our actual message is saved as grade_msg.msg, but we don't use the ".msg" extension here.
import random

def talker():
    pub = rospy.Publisher('this_msg', grade_msg, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(1) # 1hz
    while not rospy.is_shutdown():
		
		'''
		string student_name
		float32 avg_score
		bool is_passing  
		'''
		myMessage = grade_msg()
		myMessage.student_name = 'Bob'
		myMessage.avg_score = random.random()*100 
		if (myMessage.avg_score < 60):
			myMessage.is_passing = False
		else:
			myMessage.is_passing = True
			
		rospy.loginfo(myMessage)
		pub.publish(myMessage)
		rate.sleep()
	
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
        
        
