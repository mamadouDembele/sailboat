Autonomous-Sailboat
===================

My internship deals with this problem of controlling sailboat robots, which I am working on with the University of Plymouth. The first part of this internship consists of making a numerical simulation on a computer. The ROS middleware and its graphical visualization tool were used to perform this simulation. The developed code was written in C++.

##Prerequisites
You need Ubuntu Linux (18.04 or 16.04 for example) and ROS melodic installed on your computer.
To install ROS, follow the instructions on the official ROS website http://wiki.ros.org/melodic/Installation/Ubuntu

Getting Started
---------------

This repository is a ROS package. Here is a quick description of the folders.

* ***launch*** : launch file allowed to run ros node.
* ***meshs*** : CAO files
* ***src*** : Scripts contained in this repository


Use this package
----------------

1. Create a ROS workspace ***mkdir -p ~/workspaceRos/src, cd ~/workspaceRos, catkin_make***.
2. In a terminal under workspaceRos/src, type ***git clone https://github.com/mamadouDembele/sailboat.git***
3. Finally type ***echo "source ~/workspaceRos/devel/setup.bash" >> ~/.bashrc***
You now have the package installed in ***workspaceRos/src/sailboat***.

4. To test the different algorithms in the src folder, open 3 terminals.
In the first terminal, type ***roscore***. This will launch the master. In the second terminal type ***rviz***. It will appear in front of you the visualization interface. Finally in the third terminal, put yourself in the launch directory with the command ***cd ~/workspaceRos/src/sailboat/launch/***. Then type for example ***roslaunch line_follow.launch*** or ***roslaunch station_keeping.launch*** to launch nodes.
Authors
-------
<h3>MAMADOU DEMBELE</h3>

License
-------

This project is licensed under the GNU General Public License v3.0.




