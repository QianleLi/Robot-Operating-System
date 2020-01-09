# Notes

## Preface

These are notes that I took while reading the book *A Gentle Introduction to ROS*, now I am not finished but I will keep this document updated. I took an introduction course about robotics in the last semester and did a little project(mostly follow the tutorial). I want to have a thorough view of ROS after reading this book. I use VMware to run a vitual machine on my computer with Ubuntu 16.04 and ROS kinetic.
The installation tutorial is not included in this note.

## Index

- [ROS Basic](Notes.md#ROS-Basic)
  - [Packages](Notes.md#Packages)
  - [Master and nodes](Notes.md#Master-and-nodes)
## ROS Basic

### Packages

Defination: A coherent collection of files that serves a specific purpose.  
To obtain a list of all of the installed ROS packages: `rospack list`  
**package.xml**: A manifest that defines each package, the directory containing this file is called package directory, which stores most of package's files.  
Find directory of a single package:  
`rospack find [package-name]`  
**Note that tab completion can be used to help input commands.**  
To view files in a package directory:  
`rosls [package-name]`  
Go to the package directory:  
`roscd [package-name]`  
[You may return to Index](Notes.md#Index)
### Master and nodes

Nodes: Small, mostly independent programs/ A running instance of ROS program.  
Nodes have to aontact with each other, ROS master facilitates this communication.  
To start a master: `roscore`  
**Note: You should keep master running for the entire time when using ROS.**
*e.g.: Start `roscore` in one terminal and open other terminals for real work.*
* To start a node: `rosrun [package-name] [executable-name]`  
* Get a list of running nodes: `rosnode list`  
* Each node has a distinct name. Nodes names are not necessarily the same as the names of the executables underlying those nodes. If you want to set the name of a node: `rosrun [package-name] [execatable-name] __name:=[node-name]`
* Get info from a node: `rosnode info [node-name]`
* Kill a node: `rosrun kill [node-name]`, you can also kill a node by using Ctrl-C technique, but you may remove dead nodes from the node list by: `rosnode cleanup`
[You may return to Index](Notes.md#Index)
