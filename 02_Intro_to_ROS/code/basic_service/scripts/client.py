#!/usr/bin/env python

# client.py
# rosrun basic_service client.py word1 word2 word3

import sys
import rospy

from basic_service.srv import word_count


def myController():

	# Wait for the the "word_count" service:
	print 'Waiting for "word_count" service to become available...'
	rospy.wait_for_service('word_count')
	print 'DONE'

	# Capture the list of words from the command line:
	words = ' '.join(sys.argv[1:])
	
	# Call the service
	print "Calling the service..."
	call_get_count = rospy.ServiceProxy('word_count', word_count)
	
	myResponse = call_get_count(words)
	
	print words, '->', myResponse.count
	
	
if __name__ == '__main__':
	try:
		myController()
	except rospy.ROSInterruptException:
		pass
        
        
