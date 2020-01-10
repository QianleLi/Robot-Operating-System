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
  - [Compile the Hello program](Notes.md#Compile-the-Hello-program)
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
### Compile the Hello program


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
    This is the final step which sets several encironmental variables that enable ROS to find the package and its new-generated executables.**You need to do this once in each terminal.**  
	`source devel/setup.bash`