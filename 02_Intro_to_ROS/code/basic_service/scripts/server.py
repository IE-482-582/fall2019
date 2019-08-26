#!/usr/bin/env python

# server.py
# rosrun basic_service server.py

import sys
import rospy

from basic_service.srv import word_count

def count_words(request):
	print "Received: ", request
	return {'count': len(request.words.split())}
	
def myServer():	
	rospy.init_node('service_server')
	
	s = rospy.Service('word_count', word_count, count_words)
	
	rospy.spin()
	
	
if __name__ == '__main__':
    try:
        myServer()
    except rospy.ROSInterruptException:
        pass
