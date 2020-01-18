# Notes

## Preface

:smirk: These are notes that I took while reading the book *A Gentle Introduction to ROS*, now I am not finished but I will keep this document updated. I took an introduction course about robotics in the last semester and did a little project(mostly follow the tutorial). I want to have a thorough view of ROS after reading this book. I use VMware to run a vitual machine on my computer with Ubuntu 16.04 and ROS kinetic.  
:smiley: The installation tutorial is not included in this note, you may refer to [wiki.ros.org/installation](http://wiki.ros.org/ROS/Installation).   
:joy: For more tutorials, you may refer to [wiki.ros.org/tutorial](http://wiki.ros.org/ROS/Tutorials)

## Index

- [ROS Basic Concept](Notes.md#ROS-Basic-Concept)
  - [Packages](Notes.md#Packages)
  - [Master and nodes](Notes.md#Master-and-nodes)
  - [Topics and messages](Notes.md#Topics-and-messages)
    - [Messages and message types](Notes.md#Messages-and-message-types)
  - [A little more](Notes.md#A-little-more)
- [Writing ROS Programs](Notes.md#Writing-ROS-Programs)
  - [Create a workspace and a package](Notes.md#Create-a-workspace-and-a-package)
  - [Compile and execute the Hello program](Notes.md#Compile-and-execute-the-Hello-program)
  - [A publisher program](Notes.md#A-publisher-program)
  - [Compile and execute the publisher](Notes.md#Compile-and-execute-the-publisher)
  - [A subscriber program](Notes.md#A-subscriber-program)
  - [Compile and execute the subscriber](Notes.md#Compile-and-execute-the-subscriber)
- [Log Messages](Notes.md#Log-Messages)
  - [Security level](Notes.md#Security-level)
  - [An example program](Notes.md#An-example-program)
  - [View log messages](Notes.md#View-log-messages)
  - [Enable and disable log messages](Notes.md#Enable-and-disable-log-messages)
- [Graph Resource Names](Notes.md#Graph-Resource-Names)
## ROS Basic Concept

### Packages

Defination: A coherent collection of files that serves a specific purpose.  
To obtain a list of all of the installed ROS packages: `rospack list`  
**package.xml**: A manifest that defines each package, the directory containing this file is called package directory, which stores most of package's files.  
* Find directory of a single package:  
`rospack find [package-name]`  
**Note that tab completion can be used to help input commands.**  
* To view files in a package directory:   
`rosls [package-name]`  
* Go to the package directory:   
`roscd [package-name]`  
[Return to Index](Notes.md#Index)
### Master and nodes

Nodes: Small, mostly independent programs/ A running instance of ROS program.  
Nodes have to aontact with each other, ROS master facilitates this communication.  
* To start a master:  
`roscore`  
**Note: You should keep master running for the entire time when using ROS.**
*e.g.: Start `roscore` in one terminal and open other terminals for real work.*
* To start a node:  
`rosrun [package-name] [executable-name]`  
* Get a list of running nodes:  
`rosnode list`  
* Each node has a distinct name. Nodes names are not necessarily the same as the names of the executables underlying those nodes. If you want to set the name of a node:  
`rosrun [package-name] [execatable-name] __name:=[node-name]`
* Get info from a node:  
`rosnode info [node-name]`
* Kill a node:  
`rosrun kill [node-name]`  
* you can also kill a node by using Ctrl-C technique, but you may remove dead nodes from the node list by:  
`rosnode cleanup`  
[Return to Index](Notes.md#Index)
### Topics and messages

**ROS nodes wants to share info will publish messages on appropriate topics, or wants to receive info will suscribe to topics. ROS master makes sure that publishers and subscribers can find each other.**  
To see this relationship visually: `rqt_graph`    
*The author's preference is change the dropsown from "Nodes only" to "Nodes/Topics(all)", and to disable all of the checkboxes except "Hide Debug".*  
[Return to Index](Notes.md#Index)
#### Messages and message types
* To get a list of active topics:   
`rostopic list`  
* To see actual messages that are being published on the given topic to the terminal:  
`rostopic echo [topic-name]`
* To measure publication rates:  
```
rostopic hz [topiic-name]   #Measure the speed of publishment
rostopic bw [topic-name]   #Measure the bandwidth consumed by those messages
#These commands are useful to verify if messages are indeed being published regularly on particular topics
```
* Inspect a topic:  
`rostopic info [topic-name]  #This will give message tupe, publisher and subscriber.`  

    **A message type tells you what info is included and how info is organized.**
* Inspect a message type:  
`rosmsg show [message-type-name]`
    This command will give you a list of fields, wach field is defined by a build-in data type and a field name, e.g.:  
`uint8 r #Undesigned 8-bit integer called r`.    

	There are composite fields which are more complicated, but I will not show them in this note.
* You can also publish messages from the command line:  
```
rostopic pub -r [rate-in-hz] [topic-name] [message-type] [message-content]  
e.g.:
rostopic pub -r 1 /turtle1/cmd_vel geometry_msgs/Twist '[2,0,0]''[0,0,0]'  
in which the message content has a composite field.
There are more options like -f, read messages from a file. 
```

    *You may notice that `rostopic echo` and `rostopic pub` is a pair of commands that you can save the messages from publishers and republish messages to subscribers later, just like `rosbag`.*  
* Understand message type names:  
```
[package-name]/[type-name]  
e.g.:
turtlesim + Color = turtlesim/Color
package-name + type-name = message-data-type
```
[Return to Index](Notes.md#Index)
### A little more

* Communication via topics is many to many.  
    *Many publishers and many subscribers cen share a single topic.*
* ROS provides a one-to-one mechanism called services.  
[Return to Index](Notes.md#Index)
## Writing ROS Programs

### Create a workspace and a package

1. Create a directory
```
cd ~
mkdir -p workspace/src
```
2. Create a package  
Change directory to /src, use the command :  
`catkin_create_pkg package_name`  
    **ROS package names only allow lowercase letters, digits and underscores.**  
3. Put the hello.cpp right next to package.xml and CMakeLists.txt.  
    **code of hello.cpp**
```
//This header defines the standard ROS classes
#include <ros/ros.h>
int main(int argc, char **argv) {
	//Initialize the ROS system
	ros::init(argc, argv, "hello_ros");  
	//Establish this program as a ROS node
	ros::NodeHandle nh;  
	//Send some output as a log message
	ROS_INFO_STREAM("Hello,_ROS!");  //Generate an informational message
}
```

    **In this code, note that:**  
	- `#include <ros/ros.h>` should be included in every ROS program.  
    - `ros::init(argc, argv, "hello_ros");` should be called once at the beginning of your program, the last paramater is a string containing the default name of the node.
	- `ros::NodeHandle nh;` create a single NodeHandle object to use throughout the program, which is the simplest technique to register your program as a node with the ROS master.
### Compile and execute the Hello program


1. Declare dependencies  
    To list dependencies, edit the CMakeLists.txt. The line needed to be edited is* `find_package(catkin REQUIRED)`  
	Dependencies on other catkin packages can be added in a COMPONENTS section on this line:  
	`find_package(catkin REQUIRED COMPONENTS [package-name])`  
	In this example, is:  
	`find_package(catkin REQUIRED COMPONENTS roscpp)`  
	Also list dependencies in package.xml:
	```
	<build_depend>[package-name]</build_depend>
	<run_depend>[package-nema]</run_depend>
	#In this example:
	<build_depend>roscpp</build_depend>
	<run_depend>roscpp</run_depend>
	#need roscpp both at build time and run time
	```
2. Declare an executable
    Add the following two lines in CMakeLists.txt to declare the executable youd'd like to create.
	```
	add_executable([executable-name] [sourse-files])
	target_link_libraries([executable-name] ${catkin_LIBRARIES})
	#In this example
	add_executable(hello hello.cpp)
	target_link_libraries(hello ${catkin_LIBRARIES})
	```
	**When I tested the above steps on ROS kinetics, I found that one more step was needed, which was adding** `include_directories(include ${catkin_INCLUDE_DIRS})` **in the CMakeLists.txt.**
3. Build the workspace
```
cd ~/workspace
catkin_make
```
You will see sth like:  
`[100%]Built target hello`
4. Source setup.bash
    This is the final step to compile the program which sets several encironmental variables that enable ROS to find the package and its new-generated executables.**You need to do this once in each terminal.**  
	`source devel/setup.bash`
5. Start the roscore  
    The first step to execute a program is always make sure that `roscore` is run in one of your terminal.
6. Start the hello program  
    Use `rosrun` command to run the program.  
	In this case, `rosrun package_name hello`  
	[Return to Index](Notes.md#Index)
### A publisher program  

This program send randomly-generated velocity commands to a turtlesim turtle. This program named *pubvel*, **Following is the code:**  
```
#include<ros/ros.h>
#include<geometry_msgs/Twist.h> //For geometry_msgs::Twist
#include<stdlib.h> //For rand() and RAND_MAX

int main(int argc, char **argv){
	//Initialize the ROS system and become a node
	ros::init(argc, argv, "publish_vilocity");
	ros::NodeHandle nh;
	
	//Create a publisher object
	ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("turtle1/cmd_vel",1000);
	
	//Seed the random number generator
	srand(time(0))
	
	//Loop at 2 Hz until the node is shut down
	ros::Rate rate(2)
	while(ros::ok()){
		//Create and fill in the message. The other four fields, as the turtle is in a 2-D space, is ignored by turtlesim, default to 0.
		geomotry_msgs::Twist msg;
		msg.linear.x=double(rand())/double(RAND_MAX);
		msg.angular.z=2*double(rand())/double(RAND_MAX)-1;
		
		//Publish the message
		pub.publish(msg);
		
		//Send a message to rosout with the details.
		ROS_INFO_STREAM("Sending random velocity command:")
		<<" linear="<<msg.linear.x
		<<" angular="<<msg.angular.z;
		
		//Wait until it is time for another iteration
		rate.sleep();
	}
}
```
**In this code, note that:**  
- Include the message type declaration: `#include<geometry_msgs/Twists.h>`  
    You need to #include this header for every message type used in the program. Each type has a corresponding C++ header file. 
	e.g.: `#include<package_name/type_name.h>`  
	The practical impact of this naming is that later in the program we will use scope resoliution operator(::) to seperate the package name from the type name. In this example, the header defines a class called *geometry_msgs::Twist*.  
- Create a publisher object: `ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("turtle1/cmd_vel",1000);`  
    Create an object that actually does the publishing work, named ros::Publisher.
	e.g.: `ros::Publisher pub=node_handle.advertise<message_type>(topic_name,queue_size);`in which:
  * `node_handle` is an object of class `ros::NodeHandle`, which is created near the start of the program. We are calling the **advertise** method of that object.
  * `message_type` should be the name of the class defined in the header. In this case, it is *geometry_msgs::Twist*.
  * `topic_name` is a string containing the name of the topic on which we want to publish. It should match names shown by `rostopic` or `rqt_graph` but wigh out leading slash. In this case, the topic name should be *turtle1/cmd_vel*.
  * `queue_size` is an integer represent the the size of the message queue for this publisher. In most case, a reasonably large value like 1000 is suitable.   
    *If you want to publish messages on multiple topics from the same node, you will need to create a seperate ros::Publish object for each topic.*  
    **It is recommanded to create one publisher for each topic and use that publisher throughout the execution of your program.**
- Create and fill in the message object
  * Create the message onject:
      `geometry_msgs::Twist msg;`
  * Fill in the message onject:
      You can use `rosmsg show` to check the message type, it has two top-level fields, wach of which contains three sub-fields. Since there is only 2D space for the turtle, turtulsim isnores other four fields.   
	  The following lines set linear velocity between 0 and 1, and set the angular velocity to -1 and 1.
	  ```
	  msg.linear.x = double(rand())/double(RAND_MAX);
	  msg.angular.z = 2*double(rand())/double(RAND_MAX)-1;
	  ```
  * Publish the message:
      Use the publish method of ros::Publisher object to publish the message. In this example, it looks like this: `pub.publish(msg)`  
- Check for node shutdown
    `ros::ok()` checks whether the program is still in "good standing" as a ROS node. It will return true **until** the node has some reaseon to shut down. There are a few ways to get ros::ok() to return false:
  * Use rosnode kill on the node.
  * Send an interrupt signal(Ctrl-X) to the program.
  * Put `ros::shutdown()` in the program.
  * Start another code with the same name.
- Control the publishing rate  
  * `ros::Rate rate(2);` controls how rapidly the loop runs. The parameter in its constructor is in unit of Hz.
  * `rate.sleep();` is used to cause a delay in the program. The duration of the delay is calculated to prevent the loop from iterating faster than the spacified rate. In this case, which is 2 Hz. You can use `rostopic hz` to confirm that regulation.  
[Return to Index](Notes.md#Index)
### Compile and execute the publisher

*The process of building pubvel is mostly the same as for hello program, but only one important difference from hello.*  
**Declaring message type dependencies**   
As pubvel uses a messsage type from the geometry_msgs package, we must declare a dependency on that package. 
1. Modify the find_package line in CMakeLists.txt:
    ```
	find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs)
	```
2. Add elements for new dependency in package.xml:
    ```
	<build_depend>geometry_msgs</build_depend>
	<run_depend>geometry_msgs</run_depend>
	```
[Review other steps to compile a program](Notes.md#Compile-and-execute-the-Hello-program)
Finally, use `rosrun package_name pubvel` to run the publisher. Don't forget to run ros core first.  
Turn on the simulator: `rosrun turtlesim turtlesim_node`, you can see turtle respond to the motion commands that pubvel publishes.  
[Return to Index](Notes.md#Index)
### A subscriber program

This is a subscriber program named subpose that subscribe to the /turtle1/pose topic. The following are its codes:  
```
//This programs shows messages from /turtle1/pose on the screen.
#include<ros/ros.h>
#include<turtlesim/Pose.h>
#include<iomanip>  //for std::setprecision and std::fixed

//A callback function Excuted each time a new pose message arrives
void poseMessageReceived(const turtlesim::Pose& msg){
	ROS_INFO_STREAM(std::setprecision(2)<<std::fixed
	  <<"position=("<<msg.x<<","<<msg.y<<")"
	  <<" direction="<<msg.theta);
}//Print out some data from the message

int main(int argc,char **argv){
	//Initialize the ROS system and become a node
	ros::init(argc,argv,"subscribe_to_pose");
	ros::NodeHandle nh;
	
	//Create a subscriber object.
	ros::Subscriber sub = nh.subscribe("turtle1/pose",1000,&poseMessageReceived);
	
	//Let ROS take over
	ros::spin();
}
```
**In this code, note that:**  
* A callback function: The subscriber doesn't know when messages will arrive. A callback function is codes that respond to incoming messages. It looks like this:  
```
void function_name(const package_name::type_name&msg){
	...
}
//package_name and type_name refer to the message class for the topic which we plan to subscribe.
```
    In this case, the callback accepts messages of type turtlesim::Pose, so the header needed is `#include<turtlesim/Pose.h>`, the message type can be learned by using `rostopic info`.
* A subscriber object: A ros::Subscriber object needs to be created to subscribe to a topic. It looks like this:   
```
ros::Subscriber sub = node_handle.subscribe(topic_name, queue_size, 
    queue_size,pointer_to_callback_function);
//node_handle object is created before
//topic_name is the topic that we want to subscribe
//queueq_size is the integer size of the message queue for this subscriber, you can use 1000 without any worry.
//get a pointer to a function by using "&", note that do NOT add () after the function after a pointer
```
* Give ROS control:  
    There are two different ways to give permision to ROS to execute the callback function:  
	- ros::spin::Once():  
        This code asks ROS to execute all of the pending cammbacks from all of the node's subscriptions, and then return control back to us.
	- ros::spin():  
	    Works like:  
		```
		while(ros::ok()){
			ros::spinOnce();
		}
		```
		
**If your program have repetitive work to do, write a loop to do other work and call** `ros::spinOnce()` **periodically to process callbacks; if not, use** `ros::spin()` **.**  
[Return to Index](Notes.md#Index)
### Compile and execute the subscriber

This program can be compiled and executed just like the publisher and hello program.  
[You may refer to the former sections](Notes.md#Compile-and-execute-the-publisher)  
Note this program has a dependency on turtlesim since it uses the *turtlesim/Pose* message type.   
[Return to Index](Notes.md#Index)

## Log Messages

As we can see from the three programs above, `ROS_INFO_STREAM` displays informative messages to the user. These messages are examples of log messages.
### Security level

In ROS, log messages are classified into five groups called **severity level**:  
```
DEBUG
INFO
WARN
ERROR
FATAL
```
[Return to Index](Notes.md#Index)
### An example program

**The following is the codes of the program.**
```
//This program periodically generates log messages at all five severity levels
#include <ros/ros.h>

int main(int argc,char **argv){
	//Initialize the ROS system and become a node.
	ros::init(argc, argv, "count_and_log");
	ros:NodeHandle nh;
	
	//Generate log messages of varying severity regularly
	ros::Rate rate(10)
	for(int i = 1;ros::ok();i++){
		ROS_DEBUG_STREAM("Counted to " << i);
		if((i % 3) == 0){
			ROS_INFO_STREAM(i << " is divisible by 3.");
		}
		if((i % 5) == 0){
			ROS_WARN_STREAM(i << " is divisible by 5.");
		}
		if((i % 10) == 0){
			ROS_ERROR_STREAM(i << " is divisible by 10.");
		}
		if((i % 20) == 0){
			ROS_FATAL_STREAM(i << " is divisible by 20.");
		}
		rate.sleep();
	}
}
```
- Generate one-time log messages:  
In some loops and frequently-called functions, log messages my be irrigating. You can use the following codes to generate log messages only once.
```
ROS_DEBUG_STREAM_ONCE("This appears only once")
ROS_INFO_STREAM_ONCE(message)
ROS_WARN_STREAM_ONCE(message)
ROS_ERROR_STREAM_ONCE(message)
ROS_FATAL_STREAM_ONCE(message)
```
- Generate throttled log messages
```
ROS_DEBUG_STREAM_THROTTLE(0.1,"This appears every 0.1 seconds")
ROS_INFO_STREAM_THROTTLE(interval,message)
ROS_WARN_STREAM_THROTTLE(interval,message)
ROS_ERROR_STREAM_THROTTLE(interval,message)
ROS_FATAL_STREAM_THROTTLE(interval,message)
```
    The interval parameter is a double, measured the minimum time in seconds.  
[Return to Index](Notes.md#Index)
### View log messages

- Console  
    DEBUG and INFO messages are printed on standard output, whereas WARN, ERROR and FATAL messages are sent to standard error.    
	You can format console messages by setting ROSCONSOLE_FORMAT. The default format is: `[${severity}][${time}]:${message}`  
	To insert details about the source code location from which the message was generate, use combination of the `${file}`, `${line}` and`${function}`.
- Messages on `rosout`  
    Every log message is also published on the topic /rosout. The message type of this topic is `rostopic_msgs/Log`.   
	The simplest way to see /rosout messages is to use `rqt_console`.
- Log files  
    A log file is generated by the rosout node. The run_id, which is a universally-unique identifier, makes it possible to distinguish logs from seperate ROS sessions. You can use `rosparam get /run_id` to get the current run_id.  
	If the log files are too large, they will be a problem if the disk is rather small. You can use `rosclean check` to see amount of disk space in the current user account consumed by ROS logs. Use `cosclean purge` to remove all of existing logs.  
[Return to Index](Notes.md#Index)
### Enable and disable log messages

ROS programs only generate log messages at the INFO level and higher by default. The minimum severith level for a node is called **logger level**. You can set the logger level by the following three methods:  
1. Set from the command line
    After starting a node, you can use the following command to set its logger level.
	```
	rosservice call /node-name/set-logger-level ros.package-name level
	//node-name is the name of the node whose logger level you want to set
	//package-name is the name of the package that owns the node.
	//level is the logger level to use for that node
	rosservice call /count_and_log/set-logger-level ros.hello DEBUG
	```
2. Set from a GUI
    Use the command `rqt_logger_level`  
3. Set from C++ code
    Use code like this:  
	```
	#include <log4cxx/logger.h>
	...
	log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME)->setLevel(
	    ros::console::g_level_lookup[ros::console::levels::Debug]
	);
	ros::console::notofyLoggerLevelsChanged();
	```
[Return to Index](Notes.md#Index)
## Graph Resource Names

