# Quiz 1 Study Guide

The quiz will cover Chapters 1--5 of the textbook, and the online ROS tutorials from the Student Workbook.

The remainder of this document will give you some guidance on where you should focus your study efforts.  **Please be advised that this is not a comprehensive list of topics.**  In other words, don't simply study this document...**study the textbook and online tutorials carefully**.

---

## Information about our computing setup
It's important that you know some basic info about your system.  Otherwise, when you search online for solutions to issues you're encountering, you won't know if you've found solutions that are relevant to your system.  For example:
- Which version of Ubuntu are we running?
- Which version of ROS are we running?
- Which version of Python are we running?

	
## Key ROS terminology
We've introduced some new (and potentially confusing) terminology thus far this semester.  It's important that you understand terms like:
- Node
- Topic
- Service
- Action
On the quiz, you could be asked to provide an explanation/definition of these terms.  Or, you could be asked to explain the differences between/among two or more of these terms.  For example: "What's the difference between a topic and a service?"
Furthermore, you will be asked to explain why it might be preferable to use a topic rather than a service (or vice versa).  
- HINT:  Be familiar with the table on P. 74 of the textbook.


## Other ROS terminology
You should be prepared to answer the following questions:
- What is `roscore`?  When do we run it?  What does it do?
- What is `catkin_ws`?  Is it a program/application?  How do we interact with this "workspace"?
- What is `rosrun`?  When do we use this?  What does it do?
- What is `roscd`?  How does this differ from the Linux command named `cd`?
- What is `rospy`?  When do we use this?  Is it a terminal command, or something we include within a Python script?
- What is the difference between a "workspace" and a "package"?
- What is the relationship between a ROS node and a Python script?
- What is `catkin_create_pkg`?  When do we use this?  In what directory do we need to be when we run this command (e.g., `~/catkin_ws/src`)?
- What is `catkin_make`?  When do we use this?  In what directory do we need to be when we run this command (e.g., `~/catkin_ws`)?


## More details on ROS topics:
- You should be familiar with the steps required to define a custom topic.  For example:
	- What is a `.msg` file?  Where should it be saved?
	- What goes into a `.msg` file?  (HINT: In class we talked about 2 columns of information -- a data type and a field name.)
	- What are the data types for topics?  (HINT:  See the table in Chapter 3.)
	- Do we need to do anything with `CMakeLists.txt` and/or `package.xml`?  Does every package contain these two files?
	- What command do we use to to compile/build/make a package?
	
- What is a "callback" (in the context of subscribing to a topic)?
- How do you control the rate at which a topic is published?  (HINT: What roles do commands like `rate = rospy.Rate(2)` and `rate.sleep()` play?)

## Other general ROS concepts
- Structurally, how can you differentiate between a custom topic and a custom service?  In other words, if you are given the contents of a `.msg` file and the contents of a `.srv` file (without being told the filenames), how can you tell which is a topic and which is a service?
- Similarly, what are the differences among `.msg`, `.srv`, and `.action` definition files?


## Basic Linux terminal commands
You should be familiar with these Linux command-line commands:
- `cd`
- `ls`
- `pwd`
- `rm`
- `pico`
- `mkdir`
- `chmod`
- `clear`
- `man` (We haven't discussed this yet in class.  Try `man ls` for example.) 
- `cp` (We might not have discussed this yet.  Try `man cp` for details.)
- `top` (HINT:  Type `q` to exit)
- `mv` (Again, we might not have discussed this.  Try `man mv` for details.)

You should also be able to answer questions like:
- What does `cd ~` do?
- Where is your "home" directory in Ubuntu?
- What does `cd ..` do?
- When do we need to use `chmod``?
- What is the difference between `ls` and `ls -al`?


--- 

## Sample Questions:
1.  Consider the following line from a Python script:
	```
	rospy.Subscriber("chatter", String, callback)
	```
	
	You should be able to answer questions like:
	- What is rospy?
	- What is "chatter"?
	- What is "String"?
	- What is "callback"?
	- What is the difference between a "topic name" and a "message type"? 
	- What does this line actually do?  What happens when a "chatter" message is received?
	

2. Consider the following line from a Python script:
	```
	pub = rospy.Publisher('chatter', String)
	```

	You should be able to answer questions like:
	- What is rospy?
	- What is "chatter"?
	- What is "String"?
	- What is the difference between a "topic name" and a "message type"?  
	- Are "callbacks" associated with **publishers**?
	- What does this line actually do?  Does it publish a message?
	
	
3. Consider the following information copied from the terminal:

	```
	student@vm:~/catkin_ws$ catkin_make
	Base path: /home/student/catkin_ws
	Source space: /home/student/catkin_ws/src
	Build space: /home/student/catkin_ws/build
	Devel space: /home/student/catkin_ws/devel
	Install space: /home/student/catkin_ws/install
	Error(s) in package '/home/student/catkin_ws/src/custom_message/package.xml':
	Error(s):
	- The manifest (with format version 2) must not contain the following tags: run_depend
	```
	
	- What was the problem?  
	- How do you fix the problem?

	
4. Consider the following information copied from the terminal:
	```
	student@vm:~/catkin_ws/src/talklisten/scripts$ rosrun talklisten talker.py 
	  File "/home/student/catkin_ws/src/talklisten/scripts/talker.py", line 12
		rospy.init_node('talker', anonymous=True)
		^
	IndentationError: unexpected indent
	```
	
	- What was the problem?  
	- How do you fix the problem?

	
5. Is it possible for a node to ... 
	- publish more than one topic?
	- subscribe to more than one topic?  
	- both publish to one topic and subscribe to a different topic?

6. Suppose we have a package named `awesome_package`, and that we want to run a ROS node defined by a Python script called `do_great_stuff.py`.  What command do we use to run this node?  (HINT:  How do you use `rosrun`?)

7. How do you stop a ROS node from running?

8. Consider the following block of Python code:
	
	```
	rate = rospy.Rate(2)
	count = 0
	while not rospy.is_shutdown():
		count += 1
		rate.sleep()		
	```
	
	- What is the value of `count` after 10 seconds?
	
9. Suppose we have a package named `something_new`.  Furthermore, suppose we have a message-definition file, named `my_message.msg`, that contains the following:

	```
	int32 someNumber
	float32 anotherNumber
	string someTest
	```

	Now, imagine we are writing a ROS node (a Python script) that will **subscribe** to a topic named `noise`, such that this topic is of the `my_message` type.

	1. Write the single line of Python code that will import our custom message type.
	2. Write the single line of Python code that will subscribe to the `noise` topic.  Use a callback function named `noiseCallback`.
	3. Write the `noiseCallback` function.  Whenever this node receives a `noise` topic, the callback function should simply print the values of `someTest` and `someNumber`

	Next, suppose we have a ROS node that **publishes** to the `noise` topic.

	1. Write the single line of Python code that will import our custom message type.
	2. Write the single line of Python code that will define the publisher for the `noise` topic.  
	3. Write the block of Python code that will publish to the `noise` topic at a rate of 4 times per second.
	
	**NOTE**:  On the quiz, you won't be asked to write blocks of code (although you may be asked to write single lines).  I might give you a block of code with a missing line and ask you to provide that missing line.	


10. Explain the difference between these two versions of a simple publisher node:

	- VERSION 1
		```
		#!/usr/bin/env python

		import rospy
		from std_msgs.msg import Int32

		rospy.init_node('topic_publisher')
		pub = rospy.Publisher('counter', Int32)
		rate = rospy.Rate(2)

		count = 0
		while not rospy.is_shutdown():
			pub.publish(count)
			count += 1
			rate.sleep()
		```	

	- VERSION 2
		```
		#!/usr/bin/env python

		import rospy
		from std_msgs.msg import Int32

		def talker():
			rospy.init_node('topic_publisher')
			pub = rospy.Publisher('counter', Int32)
			rate = rospy.Rate(2)

			count = 0
			while not rospy.is_shutdown():
				pub.publish(count)
				count += 1
				rate.sleep()
	
		if __name__ == '__main__':
			try:
				talker()
			except rospy.ROSInterruptException:
				pass	
		```	


	
	
	
	




		
