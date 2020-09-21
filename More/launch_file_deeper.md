# An more detailed introduction of launch file.

XML version: `<?xml version="1.0"?>`  
***
Setup arguments:  
`<arg name="limited" default="false"/>`    
Use argument:  
`<arg name="limited" value="$(arg limited)"/>`  
***
Launch the gazebo map:  
```
<include file="$(find gazebo_ros)/launch/empty_world.launch">
	<arg name="world_name" value="$(find mbot_gazebo)worlds/test_world.world"/>
    <arg name="debug" value="$(arg debug)" />
    <arg name="gui" value="$(arg gui)" />
    <arg name="paused" value="$(arg paused)"/>
    <arg name="use_sim_time" value="$(arg use_sim_time)"/>
    <arg name="headless" value="$(arg headless)"/>
</include>
```
***
Launch two publishers:  
```
<node name="joint_state_publisher" pkg="joint_state_publisher" type="joint_state_publisher" />
<node name="robot_state_publisher" pkg="robot_state_publisher" type="state_publisher" />
```
***
Load the URDF model of the robot:  
```
<param name="robot_description" textfile="$(find pedsim_simulator)/urdf/p3atstevens.urdf" />
```
***
Spwan a robot model in gazebo:  
```
<node name="spawn_gazebo_model" pkg="gazebo_ros" type="spawn_model" args="-urdf -param robot_description -model robot -x $(arg x) -y $(arg y) -z $(arg z) -R $(arg roll) -P $(arg pitch) -Y $(arg yaw)" respawn="false" output="screen" />
```
***
Load parameters from a yaml file:  
```
<rosparam file="$(find ur_gazebo)/controller/arm_controller_ur5.yaml" command="load"/>
```
***
Use environment value to set argument:  
For example, you can choose burger if you input `export TURTLEBOT3_MODEL=burger` in terminals to choose the `burger` model.  
```
<arg name="model" default="$(env TURTLEBOT3_MODEL)" doc="model type [burger, waffle, waffle_pi]"/>
```
***
Start a regular node:  
pkg: The name of package.  
type: The name of executable file, like `[pub_vel].cpp`.  
name: The name of the node, which will ooverwrite the name when you initiate the node in the source file.  
respawn: If `respawn="true"`, it will restart if it is terminated.  
required: If `required=“true”`, all nodes will be terminated if this one is.  
output=“log|screen”, the output will be logged or shown on screen.
```
<node name="listener1" pkg="rospy_tutorials" type="listener.py" args="--test" respawn="true" />
<node name="bar1" pkg="foo_pkg" type="bar" args="$(find baz_pkg)/resources/map.pgm" />
```
***
Group attribute:  
If you are gonna launch many nodes that share source files, you may use the group attribute to assign different namespace to them.  
```
<group ns="turtlesim1">
	<node pkg="turtlesim" name="sim" type="turtlesim_node"/>
</group>
<group ns="turtlesim2">
	<node pkg="turtlesim" name="sim" type="turtlesim_node"/>
</group>
```
Start two turtlesim without conflicts.
***
`if` and `unless`:  
1: run the code.  
0: ignore the code.  
```
<group if="0-or-1" />
. . .
</group>
```
If teleop = true, start the teleop node.  
```
<arg name="teleop" default="false"/>
<!-- Teleop -->
<node name="rqt_robot_steering" pkg="rqt_robot_steering" type="rqt_robot_steering" if="$(arg teleop)">
<remap from="/cmd_vel" to="/Pioneer3AT/cmd_vel" />
</node>
```
1: ignore the code.  
0: run the code. 
```
<group unless="1-or-0" />
. . .
</group>
```