## Chapter 1
			
- How ROS can work as a Peer to Peer system?
	- *One example of a peer-to-peer system would be two computers connected via a network.  Later this semester we'll run ROS nodes on different computers, such that these nodes/computers communicate with each other.*

- Does ROS allow Multilingual programming?
	- *Yes, in the sense that ROS nodes can be programmed in Python or C++ (and a couple of other languages).  Each node, however, will be programmed in exactly one language.  In this class, we will only program nodes in Python.*
			
- Can the same robotic system work if there are more than 1 computer lanquage involved
	- *Yes, this is possible.  However, in this class we will only use Python.*

- Because of the multilingual philosophy, can any code other than python be run in the terminal?
	- *In this chapter, "multilingual" refers to the fact that ROS supports multiple programming languages.  Running code from the terminal is not unique to ROS.  In fact, you can run numerous different applications written in numerous programming languages from the terminal.   However, in this class, we will only be using ROS with nodes written in Python.*

- What is the parameneter server?
	- *In general, the parameter server allows you to change values of parameters BEFORE you run your code.  So, instead of setting these variables within your Python code, you can provide these variable values in the command line when you run your code.  This makes your code more generic.  We will see the parameter server later this semester...don't worry about it for now.*

- What is a TCP Port?
	- *TCP stands for "transmission control protocol".  It specifies a collection of rules that allow computers to communicate.  The TCP port is an "endpoint" where the connection is linked to your computer.  It's not necessarily a physical device.  For example, your internet connection can send/receive messages over different ports using the same WiFi/ethernet connections.  We won't be doing anything with TCP this semester.  ROS takes care of the communication connections for us.*


- I would like more clarification on what THIN means and if it is important?
	- *This isn't an important concept for this class.*

- What does IPC work?
	- *IPC is "inter process communication".  The idea is that ROS allows different processes (think of these as different programs, applications, or nodes) to share data with each other.  The details of IPC are beyond the scope of this course.  For now, it's just important that you understand that a key feature of ROS is that it enables "nodes" to communicate.*
	
- What is the difference between embeded system and ROS?
	- *Embedded systems include hardware and software designed for one specific purpose.  Examples include radios, microwaves, or wi-fi routers.  Your computer is NOT an embedded system, because it is designed to do lots of different things.  (However, within your computer you will find many embedded systems).  ROS is not an embedded system, as it does not have any hardware.  You can, however, get ROS to run on some embedded systems.*


--- 

## Chapter 2

- What is name space?
	- *The basic idea is that it would be nice if you could use the same code to run on different physical hardware devices.  The example in the book (P. 22) describes a robot with two cameras.  You don't want to write a node that reads data from one camera, and a different node that reads data from the other camera.  Namespaces allow you to run the same code in parallel, with each node communicating with a different camera.  We'll see examples of this later in the semester.*
	
- How does "remapping" this function work?
	- *This concept will make more sense later in the semester when we do some examples.  For now, just be aware that the ROS tries to make it easy to re-use code for multiple purposes, and that "remapping" is a technique that ROS enables to serve this purpose.*
	
- How the various subsystems of ROS like CV and Navigation work?
	- *We'll get to these topics later in the semester.*
	
- Each node needs to be opened by a seperate terminal. Why is this necessary? What would happen if we tried to start them all in the same terminal?
	- *The short answer is that the terminal won't allow you to run multiple nodes within the same terminal.  We need separate terminals so that each node can continue to run.  If you tried to start another node within the same terminal, you'd have to stop the existing node first.*

- In simple terms, the nodes on the graphs are the circles or dots and the edges are the lines that connect them?
	- *Correct.  Edges are sometimes also called "arcs".*

- Is roscore a pre-installed service we have that runs on its own or do we have to initiate the communication?
	- *You'll have to start roscore manually (by issuing the `roscore` command in a terminal window).*

---

## Chapter 3

- What is marshal and unmarshal?
	- *I'm afraid I couldn't find where this was discussed in the textbook.  If you'll point me to a page number I'll see where these terms were used in context.*
	
- What is advertised vs published?
	- *Topics must first be "advertised".  This is an announcement to the rest of the ROS system that there is a topic that has a certain name and is of a certain type.  This just announces the existence of the topic (like giving someone a link to a Reddit forum).  After a topic has been announced, a node can actually publish to this topic.  The publication is the act of transmitting data.  This would be like going to the aforementioned reddit forum and posting a message.*
	
- Can you explain how a Bi-directional Callback works between nodes?
	- *I'm afraid I couldn't find where this was discussed in the textbook.  If you'll point me to a page number I'll see where these terms were used in context.*
		
- How do dependencies work and what do that do?
	- *ROS has hundreds (maybe thousands) of extra libraries and message types that can be used to control various aspects of various robots.  If we loaded our programs with all of these libraries, it would make our computer really slow.  So, ROS starts with a basic/core system and then expects the user to add only those libraries that are actually required for the particular application.  These are the "dependencies" of the project.  For example, in our first in-class example (the "talklisten" package) we needed the "String" message type, which is found within the "std_msgs" package.  So, we added the "std_msgs" package as a dependency.  In our second in-class example, this dependency wasn't required.*
	
	
- The counter publisher uses the symbols += 1 to say add 1 to the counter with each iteration but what does the += actually tell the system?
	- *`x += 1` is a Python shortcut for `x = x+1`.  It simply takes the current value of `x` and adds one to it.*
	
- You can find all the topics that publish a certain message type using the rostopic find function, but you must give both the package name and the message type. So this limits us to finding all the topics publishing that message type within that package. Does this mean we can only operate one package at a time and only nodes within the same package can communicate?
	- *In general, yes.  There are some ways to work around this, but those are beyond the scope of this class.  This semester we will only be working with one ROS package at a time.  This isn't an overly restrictive limitation.*
	
- Is it a callback function just what is activated when a message is received?
	- *That's a pretty fair description.*

- What does the rate.sleep() function do at the end of example 3.5?
	- *We've talked about this quite a bit in class.  You can also check out P. 33 of the textbook.  If you still have questions about this, please come see Somayeh or me.*
	
- What is the difference between rospy.spin and a regular spin function?
	- *There's not a "regular spin" function.  rospy.spin simply keeps the program running, but it does so without making your computer burn its processors in an infinite loop.  In other words, rospy.spin is a very efficient function that keeps your node alive.*

- For the naming of topics do we just have to be consistent or do the names have to make sense? It will be easier if they make sense but is it necessary?
	- *It's always a good idea to provide descriptive names, but ROS won't judge you if you choose a non-descriptive name.  Consistency is the main requirement (your topic name also shouldn't contain spaces).*

- Can't get the basics directory to work example 3.1
	- *You'll need to see Somayeh or Dr. Murray during office hours.*

---

## Chapter 4
- When do we need to build dependancy?
	- *Any time you are going to import libraries that are not part of the base/core ROS package.  I'm sorry, this isn't a very good answer.  We'll see more examples throughout the semester...hopefully things will become more clear as you get more exposure.  For now, just assume that you'll be told which dependencies are required.*
	
- What is a Service and its components (Calls, Inputs and Outputs)
	- *We're going to see this together in class this week.*	
	
- How to Implement a service
	- *We're going to see this together in class this week.*	

- How a service can be called with proxy
	- *We're going to see this together in class this week.*	

- Will the command rqt_graph show the graph of all nodes and messages for any package we are running?
	- *In theory, yes.  However, sometimes there will be some message types won't appear in the graph if they aren't active.*	

- When we display all the published and subscribed topics what are they and what do they mean? I dont recognize all of these items and we only started a few 	  	  terminals. If we are looking for something with this command in the future how will we know if we have the right thing or not?
	- *Ask me again after we've looked at rqt_graph and some of the other ROS command line tools.*

- Will we always need to edit our package.xml file for every service?
	- *Almost always.*

- So the main two ROS communication mechanisms are services and messages?
	- *Yes, although "actions" are also useful.*

---

## Chapter 5
- What changes do we need to make to CMakeList.txt in general, such like what goes with the "find package", "add_action_file" etc.?

- What is a blocking and nonblocking program?

- Defining an Action

- How to use an Action

- In example 5-1, are the codes "duration time_to_wait" and "duration time_elapsed" pre-determined codes or built in functions?

	- *I think the answers to these questions will be addressed in our next two lectures.*

---

## 1) Navigating the ROS Filesystem

---

## 2) Creating a ROS Package
- Could not update the description tag without getting a syntax error so I edited it this way instead

	```
	~~$gedit package.xml~~
	```

	The error I keep getting
	```
	bash: syntax error near unexpected token 'newline'
	```
	Got the above error for each step in step 6

	- *You'll need to see Somayeh or Dr. Murray during office hours.*


---

## 3) Building a ROS Package

---

## 4) Understanding ROS Nodes

- What terminal do I go back to to write `rosnode list`?
	- *Try opening another/new terminal and then execute that command.*
	
- I do not get the turtlesim to be renamed my_turtle after following the instructions
	- *You'll need to see Somayeh or Dr. Murray during office hours.*
	
---

## 5) Understanding ROS Topics
- `WARNING: topic [/turtle1/command_velocity] does not appear to be published yet`
	- *My guess is that you ran a `rostopic echo` command before your node that was supposed to publish that message was actually started.*
	- *You'll need to see Somayeh or Dr. Murray during office hours.*
	
- When I try to use `rostopic pub` I get an error saying invalid message type.
	- *You'll need to see Somayeh or Dr. Murray during office hours.*

- My rqt_plot is blank, even though the other terminal is running.
	- *You'll need to see Somayeh or Dr. Murray during office hours.*

---

## 6) Understanding ROS Services and Parameters

- In Section 3.2 I cleared everything to the background and it changed the color of the	background from blue to purple like it should. But then in the terminal this came up and I'm not sure what this means 
	```
	Exception in thread Thread-3 (most likely raised during interpreter shutdown): Traceback (most recent call last): File "/usr/lib/python2.7/threading.py", line 810, in __bootstrap_inner File "/usr/lib/python2.7/threading.py", line 763, in run File "/opt/ros/indigo/lib/python2.7/dist-packages/rospy/impl/tcpros_base.py", line 154, in run : 'NoneType' object has no attribute 'timeout'
	```

	- *You'll need to see Somayeh or Dr. Murray during office hours.*
	
- After putting the parameters in a yaml file, can you load the file and use these parameters in a different package, or will they only work for the turtlesim?

	- *You can copy portions of the contents of that file, but there are some headers at the top that specify particular packages.  So, you can't simply re-use the same file as-is.*
---

## 7) Creating a ROS msg and srv

---

## 8a) Writing a Simple Publisher and Subscriber (Python)
- What's the difference between running "publisher and subscriber" nodes or "server and client" nodes?

- Are there situations where you would use one pair and not the other?

	- *We'll address both of these questions in class this week.*

---

## 8b) Examining the Simple Publisher and Subscriber

---

## 9a) Writing a Simple Service and Client (Python)

---

## 9b) Examining the Simple Service and Client

---

## MISC

- When trying to update the description tag: `bash: syntax error near unexpected token newline` 
	- *You'll need to see Somayeh or Dr. Murray during office hours.*

- When trying to set the lisence tag it says "event not found"
	- *You'll need to see Somayeh or Dr. Murray during office hours.*

- Nothing in part 6 is working for me
	- *You'll need to see Somayeh or Dr. Murray during office hours.*

- Can you select your own turtle?
	- *I'm sorry, but I don't quite understand this question*

- What does load and dump do?
	- *I'm sorry, but I don't quite understand this question.  Can you provide an example?*


- What does echo do?
	- *We'll see some examples in class.  If I don't answer your question, please ask me again during class.*
	
- When do we use msg?
	- *I'm sorry, but I don't quite understand this question*

- What is `build_export_depend` equle in format 1
	- *I'm sorry, but I don't quite understand this question*

- Whats the difference between a package and directory?
	- *A "package" is specific to ROS.  We use linux directories to organize our ROS packages.  In fact, we store each ROS package within its own linux directory.*
	
- Does `ls` show the packages or directory in the file?
	- *`ls` lists the contents of the current working directory (i.e., files and directories).  `ls` is a linux command, not a ROS command.*
	
- How can we view all possible packages?
	- *Try `rospack list`*










