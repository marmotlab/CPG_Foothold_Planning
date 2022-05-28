# CPG_Foothold_planning

Joint-Space CPG for Safe Foothold Planning and Body Pose Control during Locomotion and Climbing


Requirements
------------

You need Python 2.7 or Python 3.x or later.  You can have multiple Python
versions (2.x and 3.x) installed on the same system without problems.


## Python Packages:
-Numpy

-hebi-py==1.0.0

ROS distribution is required on your system. This code has been tested on Ubuntu 18.04 with ROS Melodic having Gazebo 9 in it.




Getting Started
-----



Install ROS Melodic (Desktop-Full Install), with all its libraries and tools. Create and build a workspace. 

Clone the Yuna package into the ROS workspace inside the src folder and build the workspace. 

To use the camera plugin, we have used this repo: https://github.com/SyrianSpock/realsense_gazebo_plugin, and cloned it in our workspace inside the src folder.  

In order to run the CPG code and Visualiztion tool: **roslaunch xMonsterCPG m6_gazebo_cpg.launch**.





