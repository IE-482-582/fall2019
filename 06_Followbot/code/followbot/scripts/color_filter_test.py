#!/usr/bin/env python

import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image

class Follower:
	def __init__(self):
		self.bridge = cv_bridge.CvBridge()
		cv2.namedWindow("Original", 1)
		cv2.namedWindow("YellowMask", 1)
		cv2.namedWindow("WhiteMask", 1)
		cv2.namedWindow("HSV", 1)
		cv2.namedWindow("Visor", 1)
		
		# Open two files for writing:
		# 1) We'll store data for the entire image (640x480):
		self.outFileBig = open("image_data_big.csv",'w')
		# 2) We'll also store data for just the small window used by our mask (640x20):
		self.outFileSmall = open("image_data_small.csv",'w')
		
		# Initialize a counter so we know how many frames we've seen:
		self.frameCount = 0
		
		self.image_sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.image_callback)

		
	def image_callback(self, msg):
		
		image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
	
		hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
		
		h, w, d = hsv.shape
		
		# UNcomment the following line to see the dimensions of the image:
		# print(h, w, d)

		# Increment our frame counter:
		self.frameCount += 1
		
		# Define the range (rows) for our mask:
		search_top = 3*h/4
		search_bot = 3*h/4 + 20

		# We're only going to save the 10th frame.
		# This will give the system a few frames to stabilize.
		if (self.frameCount == 10):
			# Loop over each pixel in the image
			for row in range(0,h):
				for col in range(0,w):
					myStr = "[%d; %d; %d], " % (hsv[row, col, 0], hsv[row, col, 1], hsv[row, col, 2])
					self.outFileBig.write(myStr)
					# Only write to the "small" file if we're in the narrow range of rows
					if (row in range(search_top, search_bot)):
						self.outFileSmall.write(myStr)
						
				self.outFileBig.write("\n")
				if (row in range(search_top, search_bot)):
					self.outFileSmall.write("\n")
				
			# We're done with the data files.  Close them now.
			self.outFileBig.close()
			self.outFileSmall.close()
			print("Data Files Written.")
						
			# Save the HSV images
			# 1) We'll first save the big image:
			cv2.imwrite("my_hsv_image_big.png", hsv)
			
			# 2) We'll now save the smaller image:
			hsv_small = hsv[search_top:search_bot, 0:w]
			cv2.imwrite("my_hsv_image_small.png", hsv_small)

			print("Images Written.")
						
		
		# Yellow Mask
		# NOTE:  YOU WILL NEED TO CHANGE THESE VALUES!
		lower_yellow = numpy.array([ 10, 10, 10]) 
		upper_yellow = numpy.array([250, 250, 250])
		yellow_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
		yellow_mask[0:search_top, 0:w] = 0
		yellow_mask[search_bot:h, 0:w] = 0		
		masked_for_yellow = cv2.bitwise_and(image, image, mask=yellow_mask)
		
		# White Mask
		# NOTE:  YOU WILL NEED TO CHANGE THESE VALUES!
		lower_white = numpy.array([ 10, 10, 10]) 
		upper_white = numpy.array([250, 250, 250])
		white_mask = cv2.inRange(hsv, lower_white, upper_white)
		white_mask[0:search_top, 0:w] = 0
		white_mask[search_bot:h, 0:w] = 0		
		masked_for_white = cv2.bitwise_and(image, image, mask=white_mask)
		
		
		# Display the original image:
		cv2.imshow("Original", image )
		
		# Display the yellow-filtered image:
		cv2.imshow("YellowMask", yellow_mask ) 
		
		# Display the white-filtered image:
		cv2.imshow("WhiteMask", white_mask ) 

		# Display the HSV image:
		cv2.imshow("HSV", hsv)
		
		# Display the visible portion of the original image:
		image_visor = image[search_top:search_bot, 0:w]
		cv2.imshow("Visor", image_visor)
		
		cv2.waitKey(3)
		
		
	
rospy.init_node('follower')
follower = Follower()
rospy.spin()

