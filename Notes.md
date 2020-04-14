# Notes

## Preface

:smirk: These are notes that I took while reading the book *A Gentle Introduction to ROS*. I took an introduction course about robotics in the last semester and did a little project(mostly follow the tutorial). I want to have a thorough view of ROS after reading this book. I use VMware to run a vitual machine on my computer with Ubuntu 16.04 and ROS kinetic.  
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
- [Launch Files](Notes.md#Launch-Files)
  - [Use launch files](Notes.md#Use-launch-files)
  - [Create launch files](Notes.md#Create-launch-files)
  - [Launch nodes inside a namespace](Notes.md#Launch-nodes-inside-a-namespace)
  - [Remapping names](Notes.md#Remapping-names)
  - [Other launch file elements](Notes.md#Other-launch-file-elements)
- [Parameters](Notes.md#Parameters)
  - [Access parameters from the command line](Notes.md#Access-parameters-from-the-command-line)
  - [Access parameters from C++](Notes.md#Access-parameters-from-C++)
  - [Set parameters in launch files](Notes.md#Set-parameters-in-launch-files)
- [Services](Notes.md#Services)
  - [Find and call services from the command line](Notes.md#Find-and-call-services-from-the-command-line)
  - [A client program](Notes.md#A-client-program)
  - [A server program](Notes.md#A-server-program)
    - [Run and improve the server program](Notes.md#Run-and-improve-the-server-program)
- [Record and replay messages](Notes.md#Record-and-replay-messages)
  - [Record and replay bag files](Notes.md#Record-and-replay-bag-files)
  - [Bags in launch files](Notes.md#Bags-in-launch-files)
- [STDR Simulator](Notes.md#STDR-Simulator)
- [Gazebo](Notes.md#Gazebo)
  - [Build a world](Notes.md#Build-a-world)
  - [Build a robot model](Notes.md#Build-a-robot-model)
  - [Additional Info](Notes.md#Additional-Info)
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
Nodes have to contact with each other, ROS master facilitates this communication.  
**Nodes shoule be largely independent of the other nodes.**
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
    	If you only use one catkin workspace, you can add this command to your `.bashrc` file, so that it automatically occurs for every new shell or terminal window you run.  
	For example, if your catkin workspace is `catkin_ws` in your home directory, edit `.bashrc` in your home directory and add the following line to the end of the file. The `.bashrc` file is normally hidden, but you can still reference it to edit it or enable "Show hidden files" in the file browser preferences.  
	`. ~/catkin_ws/devel/setup.bash`  
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
	
**This section shows how ROS resolves the name of nodes, topics, parameters and services.**  
### Global names

Examples:  
```
/teleop_turtle
/turtlesim
/turtle1/cmd_vel
```
Global names make sense anywhere they're used. They are composed of:  
* A leading slash.
* A sequence of zero or more namespaces
* A base name
### Relative names

Example:
```
/teleop_turtle
/turtlesim
/turtle1/pose
```
* Resolve relative names:  
    [default namespace]+[relative name]=[global name]
    e.g.:/turtle1 + cmd_vel = /turtle1/cmd_vel
* Set the default namespace
  - Most ROS program accept a command line parameter called __ns, which specify a default namespace for the program.
      e.g.: `__ns:default-namespace`
  - You can also set the default namespace for every ROS program using an environment variable.  
      `export ROS_NAMESPACE=default-namespace`  
	  This environment variable is used only when no other default namespace is specified by the __ns parameter.
**Relative names are used to make it easier for people to share their programs and modify them.**
### Private names

Priivate names rely on ROS client library to resolve the name to a complete global name, they are related only to its own node, and are not interested to other nodes.
[node name]+[private name]=[global name]
e.g.: /sim1/pubvel + ~max_vel = /sim1/pubvel/max_vel

### Anonymous names

Anonymous names are used to make it easier to obey the rule that each node must have a unique name.  
A node can request a unique name that is assigned automatically by passing `ros::init_options::AnonymousName as a fourth parameter.`  
e.g.: `ros::init(argc,argv,base_name,ros::init_options::AnonymousName);`  
So that we can run as many simultaneous copies of that program as we like.  
[Return to Index](Notes.md#Index)
## Launch Files

We can use launch files to specify and condigure the nodes to be used.
### Use launch files

Following is a launch file that starts the turtlesim simulator, teleoperation node and subscriber node. This should be saved as exa,ple.launch in the main package directory for the package_name package.
```
<launch>
  <node
    pkg="turtlesim"
	type="turtlesim_node"
	name="turtlesim"
	respawn="true"
  />
  <node
    pkg="turtlesim"
	type="turtle_teleop_key"
	name="teleop_key"
	required="true"
	launch-prefix="xterm -e"
  />
  <node
    pkg="package_name"
	type="subpose"
	name="pose_subscriber"
	output="screen"
  />
</launch>
```
* To execute a launch file, use this command:  
    `roslaunch package-name launch-file-name`  
    In this example, the command is:  
    `roslaunch package_name example.launch`  
    `roslaunch` will determine whether `roscore` is already running, if not, start it automatically.  
    The order of nodes started in a launch file is not certain.  
* Request verbosity
    `roslaunch` has an option to request verbose output:  
	`roslaunch -v package-name launch-file-name`  
	This command will show the details of the launch process.
* End a launched session
    Use Ctrl-C.  
[Return to Index](Notes.md#Index)
### Create launch files

- Where to place?  
    Directly in the package directory
- Basic ingredients  
  * Insert the root element
      Launch files are XML files, and every XML file has one root element. For ROS launch files, the root element is defined by a pair of launch tags:  
	  ```
	  <launch>
	    ...
	  </launch>
	  ```
	  All of other elements of each launch file should be enclosed between these tags.
  * Launch nodes
      A node element looks like this:
	  ```
	  <node
	    pkg="package-name"
		type="executable-name"
		name="node-name"
	  />
	  ```
	  A node element has three required attributes:
	  1. The pkg and type attributes identify which program ROS should run to start this node.
	  2. The name attribute assigns a name to the node. To use an anonymous name from within a launch file, the name attribute should like this:  
	      `name="$(anon [base_name])"`  
		  Note that multiple use of the same base name will generate the same anonymous name.
  * Find node log files
      Standard output from launched nodes is redirected to a log file and does not appear on the console. The name of the log file is:  
	  `~/.ros/log/[run_id]/[node_name]-number-stdout.log`  
	  The command to get run_id can be find [here](Notes.md#View-log-messages). The numbers in these file names are small integers that numbers the nodes.  
	  These log files can be viewed with text editor.
  * Direct output to the console
    - Use the output attribute in its node element:
	    `output="screen"`  
	    The example launch file above uses this attribute for subpose node, which explains why the INFO messages from this node appear on the console.  
	- Use --screen command-line option to force roslaunch to display output from all of its nodes.  
	    `roslaunch --screen package-name launch-file-name`  
		You can use this command to verify the output of your launch files.
  * Resquest respawning  
      Ask roslaunch to restart a node in the launch file when the node is terminated by adding this attribute:  
	  `respawn="true"`  
  * Requiring nodes  
      An alternative to respawn is to declare that a node is required:  
	  `required="true"`  
	  When a required node terminates, roslaunch responds by terminating all of other active nodes and exiting itself. This attribute is useful for the core node that the entire section should be abandoned without it.
  * Launch nodes in their own windows   
      When using roslaunch, all nodes share the same terminal. For nodes that rely on console input, they may be preferable to retain the seperate terminals.
	  To achieve this effect, use the `launch-prefix` attribute of a node element:  
	  `launch-prefix="command-prefix"`  
	  roslaunch will insert the given prefix at the start of the command line it constructs internally to execute the given node.
	  For example:  
	  `launch-prefix="xterm -e"`  
	  This attribute makes this node element is roughly equivalent to this command: `xterm -e rosrun turtlesim turtle_teleop_key`, in which the `xterm` command starts a simple terminal window, the `-e` argument tessl `xterm` to execute the reminder of its command line inside itself.  
[Return to Index](Notes.md#Index)
### Launch nodes inside a namespace

The ususl way to set the default namespace for a node is to use a launch file and assign the `ns` attribute in its node element:  
`ns="namespace"`  
The following codes use this attribute to create two independent turtlesim simulators.  
```
<launch>
  <node
    name="turtlesim_node"
	pkg="turtlesim"
	type="turtlesim_node"
	ns="sim1"
  />
  <node
    pkg="turtlesim"
	type="turtle_teleop_key"
	name="teleop_key"
	required="true"
	launch-prefix="xterm -e"
	ns="sim1"
  />
  <node
    name="turtlesim_node"
	pkg="turtlesim"
	type="turtlesim_node"
	ns="sim2"
  />
  <node
    pkg="package_name"
	type="pubvel"
	name="velocity_publisher"
	ns="sim2"
  />
</launch>
```
The node names in the launch file are relative names. The global name of first turtlesim_node is /sim1/turtlesim_node, the global name of second turtlesim_node is /sim2/turtlesim_node.  
**Before we discussed a situation that multiple publishers and subscribers who publish or subscribe to same type of messages. In that case, a message from one publisher will be subscribed to all subscribers. But in this section if we start the launch file above, the two simulators is truly independent, enable us to publish different velocity command to each one.**  
[Return to Index](Notes.md#Index)
### Remapping names

Each remapping provides an original name and a new name. Each time a node uses any of its remappings' original names, the ROS client library silently replaces it with the new name from that remapping.  
**Creating remappings**  
There are two ways creating remapping when starting a node:  
1. Remap a name when starting a node from the commmand line:  
    For example, run a turtlesim that ublishes its pose data on a topic named /tim instead of /turtle1/pose:  
	`rosrun turtlesim turtlesim_node turtle1/pose:=tim`  
2. Remap names with a launch file, use a remap element:  
    `<remap from="original-name" to="new-name"/>`  
	If it appears at the top level, as a child of the launch element, this remapping will apply to all subsequent nodes. Put it under a node label so that it will appear as a child of that node and only apply to the single node.  
[Return to Index](Notes.md#Index)
### Other launch file elements 

- Include other files  
    To include contents of other launch file,including all of its nodes and parameters, use an include element:  
	`<include file="path-to-launch-file"/>`  
	The file attribute expects the full path to the file we want to include. Most include elements use a `find` substitution to search for a package, instead of explicitly naming a directory.  
	`<include file="$(find package-name)/launch-file-name"/>`  
	The include element also supports the `ns` attribute for pushing its contents into a namespace:  
	`<include file="..." ns="namespace"namespace"/>`  
- Launch arguments  
    The advantage of using launch arguments is that you can avoid code duplication by writing launch files that use arguments for the small number of details that might change from run to run. The following code uses one argument called use_sim3, to determine wheter to start three copies of turtlesim or only two.  
	```
	#file name: triplesim.launch
	<launch>
	  <include
	    file="$(find package_name)/doublesim.launch"
	  />
	  <arg
	    name="use_sim3"
		default="0"
	  />
	  <group ns="sim3" if="$(arg use_sim3)">
	    <node
		  name="turtlesim_node"
		  pkg="turtlesim"
		  type="turtlesim_node"
		/>
		<node 
		  pkg="turtlesim"
		  type="turtle_teleop_key"
		  name="teleop_key"
		  required="true"
		  launch-prefix="xterm -e"
		/>
	  </group>
	</launch>
	```
  * Declare arguments  
    `<arg name="arg-name"/>`
  * Assign argument values  
    `roslaunch package-name launch-file-name arg-name:=arg-value`  
	You can also provide a value as part of `arg` declaration, using one of these syntaxes:  
	```
	<arg name="arg-name" default="arg-value"/>   #Command argument can override a default
	<arg name="arg-name" value="arg-value"/>   #cmd cannot override a value
	```
  * Use the argument value in the launch file  
    `$(arg arg-name)`  
  * Send argument values to included launch files  
    Arguments are not inherited by included launch files. The solution is to insert `arg` elements as children of the included element, like this:  
	```
	<include file="path-to-launch-file">
	  <arg name="arg-name" value="arg-value"/>
	  ...
	</include>
	```
	**Note the purpose is to establish values for the arguments needed by the included file, the value attribute is required in this context.**  
  * Create groups  
    **Group element provides a convenient way to organize nodes in a large launch file.**  
    - Group can push several nodes into the same namespace
	  ```
	  <group ns="namespace">  #Every node starts within the grup starts with the given default namespace.
	    ...
      </group>
	  ```
	  *If a grouped node has its own `ns` attribute and that name is a relative name, then the resulting node will be started with a default namespace that nests the latter namespace within the former. This rule also applies to nested groups.*
	- Groups can conditionally eable or disable nodes.  
	  ```
	  <group if="0-or-1">  #If the value of the if attribute is 0, then the enclosed elements are ignored.
	    ...
	  </group>
	  ```
	  The `unless` attribute works similarly, but with the meanings reversed:  
	  ```
	  <group unless="1-or-0">  
	    ...
	  </group>
	  ```
	  **Note that** `group` **is never strictly necessary. It is always possible to write the** `ns`**,** `if` **and** `unless` **attributes maunally for each elements that we might otehrwise include in a group. By the way, only these three attributes can be passed down via a group.**  
[Return to Index](Notes.md#Index)
## Parameters  

A centralized parameter server keeps track of a collection of values. Parameters are most suitable for configuration information that will not change much over time. All parameters are owned by the parameter server rather than by any particular node, so that parameters will continue to exist even after the node they're intended for has terminated.
### Access parameters from the command line

* Listing parameters  
    To see a list of all existing parameters, use this command:  
	`rosparam list`  
* Querying parameters  
    Ask the parameter server for the value of a parameter:  
	```
	rosparam get [parameter_name]
	rosparam get [namespace]  #Get values of every parameter in a namespace
	rosparam get /  #Get values of every parameter
	```
* Setting parameters
    `rosparam set [parameter_name] [parameter_value]`  
	This command can modify the values of existing parameters or create new ones. e.g.:  
	``` 
	rosparam set /duck_colors/huey red
	rosparam set /duck_colors/dewey blue
	rosparam set /duck_colors/louie green
	rosparam set /duck_colors/webby pink
	```
	or:  
	```
	rosparam set /duck_colors"huey: red
	dewey: blue
	louie: green
	webby: pink"
	```
* Create and load parameter files  
    Store all of the parameters from a namespace in YAML format to a file:  
	`rosparam dump [filename] [namespace]`  
	Read parameters from a file and adds them to the parameter server:  
	`rosparam load [filename] [namespace]`  
	For both commands above, the namespace parameter is optional and defaults to the global namespace. The combination of `dump` and `load` can be useful for testing.  
[Return to Index](Notes.md#Index)
### Access parameters from C++
```
void ros::param::set([parameter_name],[input_value]);
bool ros::param::get([parameter_name],[output_value]);
```
The parameter name is a string, which can be a global, relative or private name.The input value for `set` can be std::string, a bool, an int or a double. The output value for `set` should be a variable of one of these types.  
The `get` function returns true if the value was read successfully and false if there was a program.  
The following is two examples: The first example illustrates ros::param::set, and the second shows an example of ros::param::get.
```
//This program waits for a turtlesim to start up, and change its background color
#include<ros/ros.h>
#include<std_srvs/Empty.h>
int main(int argc, chat **argv){
	ros::init(argc,argv,"set_bg_color");
	ros::nodeHandle nh;
	
	//Wait until the clear service is available, which indicated that turtlesim has started up, and has set the background color parameters.
	ros::service::waitForService("clear");
	
	//Set the background color for turtlesim, overriding the default blue color.
	ros::param::set("background_r",255);
	ros::param::set("background_g",255);
	ros::param::set("background_b",0);
	
	//Get turtlesim to pick up the new parameter values
	ros::ServiceClient clearClient = nh.serviceClient<std_srvs::Empty>("/clear");
	std_srvs::Empty srv;
	clearClient.call(srv);
}
```
Before running the second example, set a parameter called `max_vel` in its private namespace: `rosparam set /publish_velocity/max_vel 0.1`  
```
//This program publishes random velocity commands, using a maximun linear valocity read from a parameter.
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <stdlib.h>
int main(int argc, char **argv) {
	ros::init(argc, argv, "publish_velocity");
	ros::NodeHandle nh;
	ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("turtle1/cmd_vel",1000);
	srand(time(0));
	
	//Get the maximum velocity parameter
	const std::string PARAM_NAME = "~max_vel";
	double maxVel;
	bool ok = ros::param::get(PARAM_NAME, maxVel);
	if(!ok){
		ROS_FATAL_STREAM("Could not get parameter"<<RAPRM_NAME);
		exit(1);
	}
	ros::Rate rate(2)
	while(ros::ok()){
		//Create and send a random velocity command
		geometry_msgs::Twist msg;
		msg.linear.x = maxVel*double(rand())/double(RAND_MAX);
		msg.angular.z = 2*double(rand())/double(RAND_MAX)-1;
		pub.publish(msg);
		
		//Wait until it is time for another iteration.
		rate.sleep();
	}
}
```
[Return to Index](Notes.md#Index)
### Set parameters in launch files

* Set parameters    
    Use a param element:  
	`<param name="param-name" value="param-value"/>`  
	The following launch file fregment does the same job as the code we illustrate `rosparam set`.  
	```
	<group ns="duck_colors">
	  <param name="huey" value="red"/>
	  <param name="dewey" value="blue"/>
	  <param name="louie" value="green"/>
	  <param name="webby" value="pink"/>
	</group>
	```
* Set private parameters  
    You can also include `param` elements as children of a node element.  
	```
	<node...>
	  <param name="param-name" value="param-value"/>
	  ...
	</node>
	```
	The parameter names are treated as private names for that node.
	e.g.:  
	```
	<node
	  pkg="package_name"
	  type="pubvel_with_max"
	  name="publish_velocity"
	/>
	  <param name="max_vel" value="3" />
	</node>
	```  
* Read parameters from a file  
    Launch files support an equivalent to `rosparam load`, to set many parameters at once.  
	`<rosparam command="load" file="path-to-param-file" />`  
	It is a typical to use a `find` substitution to specify the file name relative to a package directory.  
	```
	<rosparam
	  command="load"
	  file="$(find package-name)/param-file"
	/>
	```
	use `rosparam dump` to create the file and record past parameters, use `rosparam load` to reload the parameters, which is helpful for testing.  
[Return to Index](Notes.md#Index)
## Services

Service calls are alternative method of communication. It can differs from messages in two ways:  
1. Services calls are bi-directional. One node sends infomation to aother node and waits for a response.
2. Service calls implement one-to-one communication.  

A client node sends a request to a server node and waits for a reply. The server received the request, takes some action and sends a response back to the client.
### Find and call services from the command line
**Note the different between** `rosservice` **and** `rossrv`**.**  

| |Topics|Services|
|---|---|---|
|active things|rostopic|rosservice|
|data types|rosmsg|rossrv|

* Get a list of services that are currently active:
    `rosservice list`  
* List services by node:  
    `rosnode info node-name`  
* Find the node offering a service
    `rosservice node service-name`  
	Note that a service data type has two parts, package name and type name.  
	turtlesim + Spawn = turtlesim/Spawn
	package name + type name = service data type
* Inspect service data type:  
    `rossrv show service-data-type-name`  
	e.g.:  
	`rossrv show turtlesim/Spawn`  
	You may see something like:  
	```
	float32 x
	float32 y
	float32 theta
	string name
	---
	string name
	```
	The data before the dashes are the elements of the **request**, everything after the dash is the **response**.
	**Both request and response can be empty.**
* Call services from the command line  
    `rosservice call service-name request-content`  
	e.g.:  
	`rosservice call /spawn 3 3 0 Mikey`  
	Effect of this command is to create a new turtle named "Mikey" at position(x,y)=(3,3), facing angle θ = 0. The new turtle comes with its own set of resources which live in a namespace called Mikey.  
[Return to Index](Notes.md#Index)
### A client program

This following example illustrates all of the basic elements of a service client program.
```
//spawn_turtle.cpp
//This program spawns a new turtlesim turtle by calling the appropricte service
#include <ros/ros.h>

//The srv class for the service
#include <tutrlesim/Spawn.h>

int main(int argc, char **argv){
	ros::init(argc,argv,"spawn_turtle");
	ros::NodeHandle nh;
	
	//Create a client object for the spawn service. This will need to know the data rype of the service and its name
	ros::ServiceClient spawnClient = nh.serviceClient<turtlesim::Spawn>("spawn");
	
	//Create the request and response objects
	turtlesim::Spawn::Request req;
	turtlesim::Spawn::Response resp;
	
	//Fill in the request data members
	req.x = 2;
	req.x = 3;
	req.theta = M_PI / 2;
	req.name = "Leo";
	
	//Actually call the service. This won't return until the service is conplete.
	bool success = spawnClient.call(req, resp);
	
	//Check for success and use the response.
	if(success){
		ROS_INFO_STREAM("Spawned a turtle named "<<resp.name);
	} else {
		ROS_ERROR_STREAM("Failed to spawn.");
	}
}
```
**In summary:**  
1. Declare the request and response types  
    Like message types,every service data has an associated C++ header file that we must include: `#include <package_name/type_name.h>`  
	In this example: `#include <turtlesim/Spawn.h>`  
2. Create a client object  
    `ros::ServiceClient client = node_handle.serviceClient<service_type>(service_name)`
3. Create request and response object  
    Create a request and a response object to contain the data to and from the server.  
    ```
	package_name::service_type::Request
	package_name::service_type::Response
	```
4. Call the service  
    `bool success = service_client.call(request,response);`  
	This method does the actual work of locating the server code, transmitting the request data, waiting for a response and storing the response data. It returns a bolean value that tells us if the service call completed successfully. **It is recommend to check the return value of `call`.**
5. Declare a dependency  
    [Review steps to declare a dependency](Notes.md#Compile-and-execute-the-Hello-program)  
	For this example, modify one line in CMakeList.txt:  
	`find_package(catkin REQUIRED COMPONENTS roscpp turtlsim)`  
	Modify the package.xml:  
	```
	<build_depend>turtlesim</build_depend>
	<run_depend>turtlesim</run_depend>
	```
	After making these changes, use `catkin_make` to compile the program.  
[Return to Index](Notes.md#Index)
### A server program

The following example shows a service called toggled_forward and also drives a turtlesim robot.  
```
//pubvel_toggle.cpp
//This program toggles between rotation and translation commands, based on calls to a service
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Twist.h>

bool forward = true;
bool toggleForward(
  std_srvs::Empty::Request &req,
  std_srvs::Empty::Response &resp
){
  forward = !forward;
  ROS_INFO_STREAM("Now sending " << (Forward ? "forward" : "rotate")<< " commands.");
  return true;
}

int main(int argc, char **argv){
	ros::init(argc,argv,"pubvel_toggle");
	ros::NodeHandle nh;
	
	//Register our service with the master.
	ros::ServiceServer server = nh.advertiseService(
	  "toggle_forward",
	  &toggleForward
	);
	  
	//Publish commands, using the latest value for forward, until the node shuts down.
	ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("turtle1/cmd_vel",1000);
	ros::Rate rate(2);
	while(ros::ok()){
		geometry_msgs::Twist msg;
		msg.linear.x = forward ? 1.0 : 0.0;
		msg.angular.z = forward ? 0.0 : 1.0;
		pub.publish(msg)
		ros::spinOnce();
		rate.sleep();
	}
}
```
Sth you should know about the codes:  
* Write a service callback  
    ```
	bool function_name(
	    package_name::service_type::Request &req,
		package_name::service_type::Response &resp
	){
		...
	}
	```
	ROS executes the callback function once for each service call that the node receives. The callback function should return true to indicate success or false to indicate failure.
* Create a server object  
    We must `advertise` the service to associate the callback function with a service name and to offer the service to other nodes.  
	```
	ros::ServiceServer server = node_handle.advertiseService(
	    service_name,   //string name of service we would like to offer, can be a global name or a relative name.
		pointer_to_callback_function
	);
	```
	[A quick introduction to function pointers is included in this section.](Notes.md#A-subscriber-program)  
* Give ROS control  
    Use `ros::spin()` or `ros::spinOnce()`  
	[The discussion of differences between the two above codes.](Notes.md#A-subscriber-program)  
	
[Return to Index](Notes.md#Index)
#### Run and improve the server program

Compile and run both turtlesim_node and pub_vel_toggle. You can call the toggle_forward service from the command line to switch the motion commands:  
`rosservice call /toggle_forward`  
Because the rate of the program is set to be 2Hz, there is a noticeable lag when running it.  
Possible improvements:  
1. Use two seperate threads.
2. replace `sleep/ros::spinOnce` loop with a `ros::spin`, and use a timer callback to the publish messages.  

[Return to Index](Notes.md#Index)
## Record and replay messages

With `rosbag` we can record the messages published on one or more topics to a file and then later replay those messages.  
### Record and replay bag files

- Record bag files   
    ```
    rosbag record -O filename.bag topic-names  
	rosbag record -a ...  //record messages on every topics
	rosbag record -j ...  //enable compression in the bag file
	```
	To finish recording, use `Ctrl-C` to stop `rosbag`.  
- Replay bag files  
	`rosbag play filename.bag`  
- Inspect bag files  
	`rosbag info filename.bag`  
[Return to Index](Notes.md#Index)
### Bags in launch files  

A record node may like this:  
```
<node 
	pkg="rosbag"
	name="record"
	type="record"
	args="-O filename.bag topic-names"
/>
```
A play node may like this:  
```
<node
	pkg="rosbag"
	name="play"
	type="play"
	args="filename.bag"
/>
```
[Return to Index](Notes.md#Index)  
<<<<<<< HEAD
## STDR Simulator

I am doing my midterm project and I find this package that can easily simulate one or multiple robots navigating in 2D environment.  
The detailed tutorial can be find on [ROS Wiki](http://wiki.ros.org/stdr_simulator/Tutorials), or [ROS小课堂](https://www.corvin.cn/posts).   
Here I only want to take a note about how to draw my own map and use that map in STDR simulator.  
- First you should draw a map. For example I used GIMP to paint my map, just put on some rectangles to represent static obstacles. Then you have to save the .png file in this path: `/opt/ros/kinetic/share/stdr_resources/maps`. 
- Under the same path, you should create a new .yaml file. The content is explained in [this website](http://wiki.ros.org/map_server#YAML_format).  
- Then you should create a new .launch file, its content is like this:  
	```
	<launch>
		
		<include file="$(find stdr_robot)/launch/robot_manager.launch" />
		
		<node type="stdr_server_node" pkg="stdr_server" name="stdr_server" output="screen" args="$(find stdr_resources)/maps/sparse_obstacles.yaml"/>
	 
		<node pkg="tf" type="static_transform_publisher" name="world2map" args="0 0 0 0 0 0  world map 100" />
		
		<include file="$(find stdr_gui)/launch/stdr_gui.launch"/>
		
		<node pkg="stdr_robot" type="robot_handler" name="$(anon robot_spawn)" args="add $(find stdr_resources)/resources/robots/pandora_robot.yaml 1 2 0" />
	 
	</launch>
	```
	Note that in the above code, we loaded a map called `sparse_obstacles.yaml`. We also loaded a robot, the robot model is pandora robot, its initial location is (1,2), orientation is 0.  
In my project I took the above three steps and created three files, which is midterm1.png, midterm1.yaml and midterm1.launch. Use this command to start the simulator:  
`roslaunch stdr_launchers midterm1.launch`  
[Return to Index](Notes.md#Index)  

## Gazebo

I am doing the final project of EE631 now, in which gazebo is used to visualize the simulation. Therefore, here is the notes that I took about gazebo. By the way, in this final project, I used ubuntu 18.04 and ROS melodic, with a [Human Robot Interaction Environment](https://github.com/ral-stevens/CPE631Final) provided by the professor.  
### Build a world

To build the world in the GUI, the tutorial is [here](http://gazebosim.org/tutorials?tut=build_world&cat=build_world).  
**Note:** when running `gazebo` in a terminal and click the button 'save world as' to save the world, I encountered a problem that I cannot open the window to select a folder. To solve this problem, gazebo should be started by using `sudo gazebo` command.  
[Return to Index](Notes.md#Index)  
### Build a robot model

In this project, the robot model is described in a SDF(Simulation Description Format) file. A detailed tutorial of a SDF file can be found [here](http://gazebosim.org/tutorials?tut=build_model&cat=build_robot).  
To build a mobile robot, the tutorial can be found [here](http://gazebosim.org/tutorials?tut=build_robot&cat=build_robot). 
### Additional Info

- About the pose  
	`<pose>0 0 .1 0 0 0</pose>`  
	The pose attribute has six parameters, which is x, y, z, roll(rotate around x axis), pitch(rotate around y axis), yall(rotate around z axis). The rotation direction is based on right-hand rule.
- The visualized coordinate of gazebo  
	The red axis is x axis, the green axis is y axis and the blue axis is z axis.  
[Return to Index](Notes.md#Index)  
4/14/2020