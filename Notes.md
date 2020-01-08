# Notes

## Preface

These are notes that I took while reading the book *A Gentle Introduction to ROS*, now I am not finished but I will keep this document updated. I took an introduction course about robotics in the last semester and did a little project(mostly follow the tutorial). I want to have a thorough view of ROS after reading this book. I use VMware to run a vitual machine on my computer with Ubuntu 16.04 and ROS kinetic.
The installation tutorial is not included in this note.

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
