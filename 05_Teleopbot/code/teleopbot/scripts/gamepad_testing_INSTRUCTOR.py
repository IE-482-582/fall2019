#!/usr/bin/env python

import sys
import rospy
import pygame

from geometry_msgs.msg import Twist

''' 
XBox Controller Details:
	Buttons:
		 0 -- "A"
		 1 -- "B"
		 2 -- "X"
		 3 -- "Y"
		 4 -- Left Button
		 5 -- Right Button
		 6 -- tiny button left of the XBox logo
		 7 -- tiny button right of the XBox logo
		 8 -- XBox logo
		 9 -- Push down on the left stick
		10 -- Push down on the right stick
	Axes:
		 0 -- left stick, x axis.
		 1 -- left stick, y axis.
		 2 -- left trigger.
		 3 -- right stick, x axis.
		 4 -- right stick, y axis.
		 5 -- right trigger.
	Hat:
		 0 -- D Pad (+)
'''

# -------------------------------------
# Define some global parameters here:
JOY_ID			= 1				# 0 --> 1st identified joystick; 1 --> 2nd joystick; etc.  1 seems to work well with VM.
REFRESHRATE 	= 10.0			# Hz	
VERBOSE			= True			# True --> Print info (useful when debugging). 

# FIXME:
LINEAR_AXIS 	= 1				# FIXME -- You need to choose a value for this
ANGULAR_AXIS 	= 0				# FIXME -- You need to choose a value for this
# -------------------------------------


# FIXME:  Do we want this?
class make_joy_info():
	def __init__(self):
		# Set self.joy_info
		self.linear 	= 0.0
		self.angular	= 0.0
	
class runJoystick():
	def __init__(self):
		
		# Initialize some ROS stuff here
		rospy.init_node('joystick', anonymous=True)		
		
		# Set the shutdown function
		rospy.on_shutdown(self.shutdown)		

		# How often should we send messages?
		self.rate = rospy.Rate(REFRESHRATE)	
		
		# BLOCK FOR STUDENT
		# Define the publisher to send info to a "gamepad_to_twist..." node.
		self.pub_joy_twist = rospy.Publisher('joy_twist', Twist, queue_size=1)
		
		
		# Try to connect to the joystick.
		# We'll assume we can't connect unless proven otherwise:
		couldConnect = False	
		try:			
			pygame.init()
			
			# Initialize the joysticks
			pygame.joystick.init()
						
			# Used to manage how fast the joystick is polled
			clock = pygame.time.Clock()
			
			# Get count of joysticks
			joystick_count = pygame.joystick.get_count()
			print "Number of joysticks: {}".format(joystick_count)

			# NOTE: If there are multiple joysticks, we are only going to listen 
			# to the one identified by JOY_ID.

			if (joystick_count > 0):
				joystick = pygame.joystick.Joystick(JOY_ID)
				joystick.init()
			
				print "Joystick ID: {}".format(JOY_ID)
				
				# Get the name from the OS for the controller/joystick
				name = joystick.get_name()
				print("Joystick name: {} \n".format(name) )
				
				# Axes usually run in pairs.
				# up/down for one, and left/right for the other.
				axes = joystick.get_numaxes()
				print("Number of axes: {}".format(axes) )
							
				buttons = joystick.get_numbuttons()
				print("Number of buttons: {}".format(buttons) )
				
				# Hat switch. All or nothing for direction, not like joysticks.
				# Value comes back in an array.
				hats = joystick.get_numhats()
				print("Number of hats: {}".format(hats) )

				# Trackballs (not found on an XBox joystick):
				trackballs = joystick.get_numballs()
				print("Number of trackballs: {}".format(trackballs))
			
				couldConnect = True

		except:
			print "Error with pygame.init():"
			e = sys.exc_info()[1]
			print e
			raise
			
		if (not couldConnect):
			print "Could not connect to joystick."
			# self.shutdown()
			rospy.signal_shutdown("Could not connect to joystick.")			
		
		# Give the system a little time to re-initialize:
		rospy.sleep(0.5)					

		# Initialize our data structure:
		self.joy_info = make_joy_info()
		

		# Keep this node alive.
		print "Joystick Node Running..."
		while not rospy.is_shutdown():	

			# EVENT PROCESSING STEP
			# Events are created when the user interacts with the joystick.
			for event in pygame.event.get(): 
				# Uncomment the next line if you want to see the raw "event":
				# print event

				# We'll assume that we don't have anything to publish.
				pubTwist = False
								
				# Possible joystick actions: JOYAXISMOTION JOYBALLMOTION JOYBUTTONDOWN JOYBUTTONUP JOYHATMOTION
				if (event.type == pygame.JOYBUTTONDOWN):
					if (VERBOSE):
						print("Joystick button pressed.")
						print("\t Button ID: {}".format(event.button))

				elif (event.type == pygame.JOYBUTTONUP):
					if (VERBOSE):
						print("Joystick button released.")
						print("\t Button ID: {}".format(event.button))

				elif event.type == pygame.JOYHATMOTION:
					# print event.joy	-- Will always be JOY_ID.
					# value --> (x, y).  
					# 					up:     (0,  1)
					# left: (-1, 0)		static: (0,  0)				right: (1, 0)
					# 		  			down:   (0, -1)
					# Note:  The "static" case only happens when you release the hat.
					if (event.value == (0,0)):
						if (VERBOSE):
							print("Hat released.")
							print("\t Hat ID: {}".format(event.hat))
							print("\t x = {}, y = {}".format(event.value[0],event.value[1]))
					
					else:
						if (VERBOSE):
							print("Hat pressed.")
							print("\t Hat ID: {}".format(event.hat))
							print("\t x = {}, y = {}".format(event.value[0],event.value[1]))

				elif event.type == pygame.JOYAXISMOTION:
					# print "Axis Motion"
					# joy: joystick id of the event (will always be JOY_ID)
					# axis: axis id of the event
					# value: new position of the axis, -1 (off) to 1 (fully depressed) with 0 the center	
					if (VERBOSE):
						print("Axis Motion")
						print("\t Axis ID: {}".format(event.axis))
						print("\t Value: {}".format(event.value))
										
					if (event.axis == LINEAR_AXIS):
						# up is -1, down is 1
						
						self.joy_info.linear = -1 * event.value
						# self.joy_info.angular remains unchanged

						pubTwist = True

					elif (event.axis == ANGULAR_AXIS):
						# left is -1, right is 1
						
						self.joy_info.angular = event.value
						# self.joy_info.linear remains unchanged

						pubTwist = True

					if (pubTwist):
						# Publish message to joy_twist topic
						t = Twist()
						t.linear.x  = self.joy_info.linear
						t.angular.z = self.joy_info.angular
						self.pub_joy_twist.publish(t)

				elif (event.type == pygame.JOYBALLMOTION):
					if (VERBOSE):
						print("JOYBALLMOTION")
						print(event)
						# I have no idea what info this provides...
						# We don't have a joystick that produces this event.

			# We're going to use pygame's sleep function:			
			clock.tick(REFRESHRATE)
			

	def shutdown(self):
		rospy.loginfo("Shutting down the Joystick node...")

		try:
			pygame.quit()
		except:
			print "Could not quit pygame."
			e = sys.exc_info()[1]
			print e
			
		rospy.sleep(1)
		

if __name__ == '__main__':
	try:
		runJoystick()
	except rospy.ROSInterruptException:
		rospy.loginfo("Joystick node terminated.")	
