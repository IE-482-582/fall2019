# Introduction to the Robot Operating System (ROS)

---

## NOTE:  You have work to do!

Before **we** review the remainder of this document in class, **you** need to work through the ROS tutorials.

1. Open [student_workbook.md](student_workbook.md).

2. Submit your questions at the beginning of class.
	- Also leave notes to yourself here.  Just make sure you clearly diffentiate between questions and notes.
	
---
         
   
- What is ROS?
   - ROS is a framework that makes it easy to write software for robots.  It has a large collection of libraries and tools that you don't have to re-create yourself.
   - ROS supports several simulation tools, and can also run on actual robots. 
   - It's not an actual operating system (like Windows, iOS, Linux, Android, or OSX).
   - It only runs on Linux.  We're using Ubuntu 14.04, which supports ROS Indigo.
   - ROS has evolved over several versions, which are in alphabetical order.  Some of the recent ones include:
      - Groovy | Hydro | **Indigo** | Jade | Kinetic | Lunar | Melodic

      We are using **Indigo**
   - ROS supports C++, Python, Java, and other programming languages.  We will be using Python in this class.
   
- Why are we using ROS in this course?

   - One of the goals of this course is for you to gain an appreciation for the types of tasks that can be performed by robots.  To do this, I want you to be able to experiment with robot controllers.  Without ROS, you'd have to write these controllers yourself, which would take forever.  ROS will enable you to steer simulated robots, analyze robot sensor data, coordinate multiple robots, etc. 

- Does ROS only control robots?

   - Nope.  We've also used ROS to control simulations of a food delivery service.
      - Nodes:  Restaurants, customers, and couriers
      - Messages:  Where is each courier?  When was an order placed?  When is an order available?  What are the courier assignments?
   
      <!--[INCLUDE VIDEO FROM ZACH]-->
      
- Some Terminology (A ROS Hierarchy)
   - roscore: 
   - catkin_ws
   - Package
   - nodes, topics, messages, services
                  
   - Need to compile (make) each new package (or if we change the structure of messages/services, or if we add new topics)

--- 
      
## Example #1:  Basic Talker & Listener

We're going to create a **package** named `talklisten`.

In this package there will be two **nodes**.  
One is a "talker"; it will broadcast a message at a fixed rate (like 10-times per second).  
This message will be published/transmitted via a **topic** named `chatter` (which is of the message type `String`, a standard ROS message type).

The other node is a "listener"; it will subscribe (listen) to the `chatter` topic.  
Note:  It's not really listening to the `talker` node.  It's subscribed to a topic that the talker publishes to.

<!--[INSERT DIAGRAM HERE]-->

Here's an analogous application:
Suppose the talker is a car, and that it is broadcasting its location to a topic.  Anyone subscribing to that topic can hear this information.  We could have lots of nodes that listen to this topic.

### Follow these steps:

1.  Create our `talklisten` package:
   
    - First, we'll change directories (`cd`) to the catkin workspace:

      ```
      cd ~/catkin_ws/src
      ```
   
    - Let's see what's here before we create our package:
      ```
      ls
      ```
   
    - Now let's tell ROS to create an empty package named `talklisten`:
      ```
      catkin_create_pkg talklisten
      ```

    - Let's see what's here now:
      ```
      ls
      ``` 
      You should see a directory (folder) named `talklisten`.
   
    - Change directories into `talklisten`:
      ```
      cd talklisten
      ```

    - What's in the `talklisten` directory?
      ```
      ls
      ```
   

2.  In the future, when we have more complex packages, we'll need to edit the `CMakeLists.txt` and `package.xml` files.  For now, we can just leave them as-is.

3.  It's a good idea to keep your code organized within each package.  We'll create separate directories for source code (Python scripts that represent **nodes**), for custom **message** structures, and for custom **service** structures. 

    - Let's create a directory for our source code:

      ```
      mkdir scripts
      ```

    - We won't need to create message (`msg`) or service (`srv`) directories for this project.
   
    - Let's see what we have:
      ```
      ls -al
      ```
      You should see `CMakeLists.txt`, `package.xml`, and `scripts`.

4.  Now it's time to write our `talker` node:

    - Open a text editor.  Geany is a good one.  You might also like Sublime Text, or even gedit.
   
    - Within your text editor type the following Python code:
      ```python
      #!/usr/bin/env python
      
      # license removed for brevity
      # This code is copied from
      # http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29
      
      import rospy
      from std_msgs.msg import String
      
      def talker():
          pub = rospy.Publisher('chatter', String, queue_size=10)
          rospy.init_node('talker', anonymous=True)
          rate = rospy.Rate(1) # 1hz
          while not rospy.is_shutdown():
              hello_str = "hello world %s" % rospy.get_time()
              rospy.loginfo(hello_str)
              pub.publish(hello_str)
              rate.sleep()
      
      if __name__ == '__main__':
          try:
              talker()
          except rospy.ROSInterruptException:
              pass
      ```
      
    - Save this file as `talker.py` inside the `~/catkin_ws/src/talklisten/scripts` directory.
   
    - Make sure the file is in the right place:
      ```
      cd ~/catkin_ws/src/talklisten/scripts
      ls
      ```
      If you don't see `talker.py` you've made a mistake.  Make sure you follow the instructions *exactly as written*.
   
    - We now need to make our talker script executable:
      ```
      chmod +x talker.py
      ```
   
   
5.  Now we'll write our `listener` node:

    - Open another tab in your text editor. 
   
    - Within your text editor type the following Python code:

      ```python
      #!/usr/bin/env python
      
      # license removed for brevity
      # This code is copied from
      # http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29
      
      import rospy
      from std_msgs.msg import String
      
      def callback(data):
          rospy.loginfo(rospy.get_caller_id() + " I heard: %s", data.data)
          
      def listener():
      
          # In ROS, nodes are uniquely named. If two nodes with the same
          # node are launched, the previous one is kicked off. The
          # anonymous=True flag means that rospy will choose a unique
          # name for our 'listener' node so that multiple listeners can
          # run simultaneously.
          rospy.init_node('listener', anonymous=True)
      
          rospy.Subscriber("chatter", String, callback)
      
          # spin() simply keeps python from exiting until this node is stopped
          rospy.spin()
      
      if __name__ == '__main__':
          listener() 
      ```   

    - Save this file as `listener.py` inside the `~/catkin_ws/src/talklisten/scripts` directory.
   
    - Make sure the file is in the right place:
      ```
      cd ~/catkin_ws/src/talklisten/scripts
      ls
      ```
      If you don't see `listener.py` you've made a mistake.  Make sure you follow the instructions *exactly as written*.

    - We now need to make our talker script executable:
      ```
      chmod +x listener.py
      ```


6.  It's time to *compile* our package so ROS can use it.

    - First, change directories to the catkin workspace:
      ```
      cd ~/catkin_ws
      ```
   
    - Now, compile (make) the package:
      ```
      catkin_make
      ```
   
    - If you got an error message, you'll have some debugging to do.
   
7. Finally, it's time to run our code.  We're going to need three (3) terminal windows.  

    - The 1st terminal will run `roscore`, which is the main ROS application:  
      ```
      cd ~
      roscore
      ```
   
    - The 2nd window will run our listener node.  We'll use `roscd` to change directories to our package's home directory, and then `cd` into our `scripts` directory.  Finally, we'll start the listener node.
      ```
      roscd talklisten
      cd scripts
      rosrun talklisten listener.py
      ```
   
    - The 3rd window will run our talker node.
      ```
      roscd talklisten
      cd scripts
      rosrun talklisten talker.py
      ```

    - Toggle back and forth between the 2nd and 3rd terminal windows to see what's happening.
   

### Understanding the Code
- We will review the code together in class.
- Note that we are using a message type of `String`.  Let's get some more info about it:
    ```
    rosmsg show String
    ```
    It has one field, named `data`, which is of type `string`.

---

## Example #2:  Creating a Custom Message Type

Suppose we want to transmit the following information:
- Name (a text string)
- Average Score (a floating point value)
- Passing (True/False)

There's not a built-in ROS message that has this structure.  So, we're going to create our own.

Before we get started, here's our plan:
- We'll create a new package, named `custom_message`
- We'll name our new message type `grade_msg`.  (Note:  We are going to broadcast a **topic** using this **message type**.  Make sure you understand the difference between these two.)
- We'll write a "talker" node (`talker2.py`) and a "listener" node (`listener2.py`).

### Follow these steps:
1.  Create our `custom_message` package:
    ```
    cd ~/catkin_ws/src
    catkin_create_pkg custom_message
    ```

2.  Edit the `package.xml` file **for this package**.
   
    - Use your favorite text editor to open the file. 
     
    - Add the following lines *after* `<buildtool_depend>catkin</buildtool_depend>`:
       ```
       <build_depend>roscpp</build_depend>
       <build_depend>rospy</build_depend>
       <build_depend>std_msgs</build_depend>
       <build_depend>message_generation</build_depend>
       
       <run_depend>roscpp</run_depend>
       <run_depend>rospy</run_depend>
       <run_depend>std_msgs</run_depend>
       <run_depend>message_runtime</run_depend>
       ```
       
    - Make sure to save the file.   

3.  Edit the `CMakeLists.txt` file **for this package**:   

    - Use your favorite text editor to open the file.  
    - Add `message_generation` to the end of the `find_package()` call:
       ```
       find_package(catkin REQUIRED COMPONENTS
         roscpp
         rospy
         std_msgs
         message_generation
       )
       ```

    - Add `message_runtime` to the end of the `catkin_package()` call:
       ```
       catkin_package(
         #  INCLUDE_DIRS include
         #  LIBRARIES custom_message
         CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
         #  DEPENDS system_lib
       )
       ```
       
    - Add `grade_msg.msg` to the end of the `add_message_files()` call:
       ```
       ## Generate messages in the 'msg' folder
       add_message_files(
         FILES
         #   Message1.msg
         grade_msg.msg
       )
       ```
       
    - Make sure the `generate_messages()` call is uncommented and contains all of the dependencies required by our new message:
       ```
       ## Generate added messages and services with any dependencies listed here
       generate_messages(
         DEPENDENCIES
         std_msgs  # Or other packages containing msgs
       )
       ```

    - Save the file.   

4. Create the `scripts` and `msg` directories for this package:
    ```
    cd ~/catkin_ws/src/custom_message
    mkdir scripts
    mkdir msg
    ```
    
5. Define our `grade_msg` message:

    - We'll use the `pico` text editor.
       ```
       cd ~/catkin_ws/src/custom_message/msg
       pico grade_msg.msg
       ```
       
    - In the text editor, write the following:
       ```
       string student_name
       float32 avg_score
       bool is_passing  
       ```
       
    - Save the file (`ctrl-o`) and exit (`ctrl-x`)
    
6. Write our talker:
    - Open your text editor.
    
    - Within the text editor, type the following Python code:
         ```python
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
         ```
    
    - Save this file as `talker2.py`, inside the `~/catkin_ws/src/custom_message/scripts` directory.
    

7. Write our listener
    - Open your text editor.
    
    - Within the text editor, type the following Python code:
      ```python
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
      ```
      
    - Save this file as `listener2.py`, inside the `~/catkin_ws/src/custom_message/scripts` directory.


8. Compile the package:
   ```
   cd ~/catkin_ws
   catkin_make
   ```
   
7. Run our code.  We're going to need three (3) terminal windows.  

    - The 1st terminal will run `roscore`, which is the main ROS application:  
      ```
      cd ~
      roscore
      ```
   
    - The 2nd window will run our listener node.  
      ```
      roscd custom_message
      cd scripts
      rosrun custom_message listener2.py
      ```
   
    - The 3rd window will run our talker node.
      ```
      roscd custom_message
      cd scripts
      rosrun custom_message talker2.py
      ```

    - Toggle back and forth between the 2nd and 3rd terminal windows to see what's happening.
   
      

---

## Example #3:  Implementing a Service

When we broadcast a topic, we don't wait for a response from anyone.
Services are different:
- One node (we'll name it `client.py`) will be the client.  
- Another node (`server.py`) will be the server.
- The client is going to send some info to the server, and wait for the server to reply with some response.
- In this example, the client will send a list of words; the server will reply with a count of the number of words.

As with our custom message we defined above, we're going to need a custom service.

Before we get started, here's our plan:
- We'll create a new package, named `basic_service`.
- We'll name our new service type `word_count`.  
- We'll write a "client" node (`client.py`) and a "server" node (`server.py`).

### Follow these steps:
1.  Create our `basic_service` package:
    ```
    cd ~/catkin_ws/src
    catkin_create_pkg basic_service
    ```

2.  Edit the `package.xml` file **for this package**.
   
    - Use your favorite text editor to open the file. 
     
    - Add the following lines *after* `<buildtool_depend>catkin</buildtool_depend>`:
       ```
       <build_depend>roscpp</build_depend>
       <build_depend>rospy</build_depend>
       <build_depend>std_msgs</build_depend>
       <build_depend>message_generation</build_depend>
       
       <run_depend>roscpp</run_depend>
       <run_depend>rospy</run_depend>
       <run_depend>std_msgs</run_depend>
       <run_depend>message_runtime</run_depend>
       ```
       
    - Make sure to save the file.   

3.  Edit the `CMakeLists.txt` file **for this package**:   

    - Use your favorite text editor to open the file.  
    - Add `message_generation` to the end of the `find_package()` call:
       ```
       find_package(catkin REQUIRED COMPONENTS
         roscpp
         rospy
         std_msgs
         message_generation
       )
       ```

    - Add `message_runtime` to the end of the `catkin_package()` call:
       ```
       catkin_package(
         #  INCLUDE_DIRS include
         #  LIBRARIES custom_message
         CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
         #  DEPENDS system_lib
       )
       ```
       
    - Add `word_count.srv` to the end of the `add_service_files()` call:
       ```
       ## Generate services in the 'srv' folder
       add_service_files(
         FILES
         # Service1.srv
         word_count.srv
       )
       ```
       
    - Make sure the `generate_messages()` call is uncommented and contains all of the dependencies required by our new service:
       ```
       ## Generate added messages and services with any dependencies listed here
       generate_messages(
         DEPENDENCIES
         std_msgs  # Or other packages containing msgs
       )
       ```

    - Save the file.   

4. Create the `scripts` and `srv` directories for this package:
    ```
    cd ~/catkin_ws/src/basic_service
    mkdir scripts
    mkdir srv
    ```
    
5. Define our `word_count` message:

    - We'll use the `pico` text editor.
       ```
       cd ~/catkin_ws/src/basic_service/srv
       pico word_count.srv
       ```
       
    - In the text editor, write the following:
       ```
       string words
       ---
       uint32 count
       ```
       
    - Save the file (`ctrl-o`) and exit (`ctrl-x`)
    
6. Write our client:
    - Open your text editor.
    
    - Within the text editor, type the following Python code:
         ```python
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
         ```
    
    - Save this file as `client.py`, inside the `~/catkin_ws/src/basic_service/scripts` directory.
    

7. Write our server:
    - Open your text editor.
    
    - Within the text editor, type the following Python code:
      ```python
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
      ```
      
    - Save this file as `server.py`, inside the `~/catkin_ws/src/basic_service/scripts` directory.


8. Compile the package:
   ```
   cd ~/catkin_ws
   catkin_make
   ```
   
7. Run our code.  We're going to need three (3) terminal windows.  

    - The 1st terminal will run `roscore`, which is the main ROS application:  
      ```
      cd ~
      roscore
      ```
   
    - The 2nd window will run our client node.  
      ```
      roscd basic_service
      cd scripts
      rosrun basic_service client.py blah blah
      ```
   
    - The 3rd window will run our server node.
      ```
      roscd basic_service
      cd scripts
      rosrun basic_service server.py
      ```

    - Toggle back and forth between the 2nd and 3rd terminal windows to see what's happening.
   


---


## Review

### Workflow for Creating a Package

In the steps below, replace `name_of_package` with a particular package name.

1. Tell ROS to create the package:
   ```
   cd ~/catkin_ws/src
   catkin_create_pkg name_of_package
   ```

2. Edit `CMakeLists.txt` and `package.xml` as necessary.
   
3. Create a `scripts` directory (always).  Also create `msg` and/or `srv` directories (as necessary).
   ```
   cd ~/catkin_ws/src/name_of_package
   mkdir scripts
   mkdir msg
   mkdir srv
   ```
   
4. Write your Python scripts (ROS nodes) and save them in the `scripts` directory.

    - Don't forget to make your `.py` scripts executable:
      ```
      cd ~/catkin_ws/src/name_of_package/scripts
      chmod +x my_script.py
      ```
      (Replace `my_script` with an actual Python script name.)
   
5. Compile (make) the package:
   
   ```
   cd ~/catkin_ws
   catkin_make
   ```
   
6. Run your code:
    - You'll need one terminal window open to run `roscore`:
      ```
      cd ~
      roscore
      ```
    - You'll need a separate terminal window open for each ROS node:
      ```
      cd ~/catkin_ws/src/name_of_package/scripts
      rosrun name_of_package my_script.py
      ```
      
    - When you're done, use `Ctrl-c` to stop the code running in each terminal window.
   

### Workflow for Running a Package
- When you want to run an existing package, you only need to follow the last step above.  In other words, don't re-create the package.

### When do we need to Recompile a Package?
- Sometimes you'll need to recompile an existing package.  This usually happens when one of the following has occurred:
   1. You have edited either `CMakeLists.txt` or `package.xml`.
   2. You have modified the structure of a message or a service.

### Anatomy of a Package
```
cd ~/catkin_ws/src/talklisten
tree
```

```
├── CMakeLists.txt
├── package.xml
└── scripts
    ├── listener.py
    └── talker.py
```   
   
   
   
