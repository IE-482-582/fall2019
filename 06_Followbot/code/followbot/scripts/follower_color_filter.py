#!/usr/bin/env python

import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image

class Follower:
	def __init__(self):
		self.bridge = cv_bridge.CvBridge()
		cv2.namedWindow("window1", 1)
		cv2.namedWindow("window2", 1)
		
		self.image_sub = rospy.Subscriber('camera/rgb/image_raw', 
									  Image, self.image_callback)

	def image_callback(self, msg):
		image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
		
		hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
		
		lower_yellow = numpy.array([ 10, 10, 10]) 		#	numpy.array([ 50,  50, 170])
		upper_yellow = numpy.array([255, 255, 250])
		yellow_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
		masked = cv2.bitwise_and(image, image, mask=yellow_mask)
		
		# Display the original image:
		cv2.imshow("window1", image )
		
		# Display the filtered image:
		cv2.imshow("window2", yellow_mask ) 
		
		cv2.waitKey(3)
	
rospy.init_node('follower')
follower = Follower()
rospy.spin()
# END ALL
