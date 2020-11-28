Author: Qianle Li  
qli59@stevens.edu  
Advisor: Yi Guo  
11/28/2020
***
This repository is created for the convenience of sharing and further research. It keeps a backup of my EE800 project.  
This project is consist of the RotorS Simulator, ORB-SLAM package, ground robot model package and the navigation stack. Besides, I have some necessary dependencies included in the drone_ws workspace.  
**The whole project was tested on Ubuntu18.04, ROS Melodic and Gazebo 9.**  
If you want to know more details about this project, please check my final report.  

***
# Index

* [ORBtest_ws](README.md#ORBtest_ws)
	* [ORB-SLAM Package](README.md#ORB-SLAM-Package)
* [drone_ws](README.md#drone_ws)
	* [RotorS Simulator](README.md#RotorS-Simulator)
	* [Ground Robot Model and Navigation Package](README.md#Ground-Robot-Model-and-Navigation-Package)
	* [SimpleLayer](README.md#SimpleLayer)
* [Instructions](README.md#Instructions)
	* [Build](README.md#Build)
	* [Run](README.md#Run)
	* [Notes](README.md#Notes)
***
## ORBtest_ws

### ORB-SLAM Package

**I only used stereo camera in this project, so codes for RGB-D cameras and monocular cameras are not studied and tested.**  
The original version of ORB-SLAM package is [here](https://github.com/raulmur/ORB_SLAM2).  

I added some code in System.cc, Tracking.cc, Frame.cc and KeyFrame.cc. Needless to say I also modified those related head files. You can search for `EE800 code` to find those codes I added in those source files.    

You will find plenty of commented codes in those source files. I kept them in case I might use them later. You can safely delete all those commented codes in source files.  
***
In the final version of my project, the ORB-SLAM package ared used to extract ORB points from images taken from a stereo camera. Actually, the package does much more than that, you can read the [paper](https://ieeexplore.ieee.org/abstract/document/7946260) to learn details. In short, this SLAM algorithm can be applied on UAVs, and hand-hold devices. It can build a 3D map and locate the camera in the map. Based on my own testing experiments and some comments from the Internet, ORB-SLAM works better in indoor environment. To achieve better performance, you should try to do as more close-looping as possible.  
![ORB-SLAM Configuration](https://github.com/QianleLi/EE800/blob/master/images/ORB-SLAM%20Configuration.jpg "ORB-SLAM Configuration")  
However, I don't need this package to actually do SLAM in this project. As you can see in the above picture, after its tracking process, it deterines if the current frame is a new key frame. If it decides to create a new key frame, I collect all new map points in that new key frame and publish them as a C++ vector.  
Therefore, some parts of the package is run for no purpose in my project, which is a way to improve the efficiency of the whole system. Besides, ORB-SLAM also supports RGB-D cameras and monocular cameras. I think there are still a lot to dig in application and improvement.  
[Return to Index](README.md#Index)  
## drone_ws

### RotorS Simulator

Rotors Simulator is a package developed by the Autonomous Systems Lab of ETH zurich. You may find the original package from [here](https://github.com/ethz-asl/rotors_simulator/wiki).    

If you want to build the original package, you may follow [this instruction](https://darienmt.com/autonomous-flight/2018/11/15/installing-ethz-rotors.html).  

There are models of several UAVs and [VI-sensor](http://wiki.ros.org/vi_sensor) included in this package. You may check those models in the package `rotors_description`.  

In my project, the UAV always hovering above the ground robot. To achieve that, I create a node named "syncronization" to achieve that. The node does the following jobs:  
1. Subscribe to the topic `gazebo/modelstates` to get the current location of the ground robot.  
2. Transform the message type from `gazebo_msgs::ModelStates` to `trajectory_msgs::MultiDOFJointTrajectory`.
3. During the process of transformation, change the value `z` to 2.5, which means the UAV will keep hovering at the height of 2.5 meters.
4. Publish the position message to default topic for the UAV to control it in the Gazebo environment.  
The source file is `my_wappoint.cpp` under `~drone_ws/src/rotors_simulator/rotors_gazebo/src`.  

[Return to Index](README.md#Index)  
***
### Ground Robot Model and Navigation Package

The robot model is in the `mbot_description` package, I built this robot model and it is not a real robot model.  
The robot is equipped with a RGB-D camera and a laser sensor, moving with two differential drived wheels and a caster wheel. However, the RGB-D camera is not used in this project due to its poor performance camparing to the laser sensor in mapping mission.  
`mbot_navigation` contains configuration files of the move_base package, maps, launch files for gmapping and move_base, and the configuraton of rviz. 

[Return to Index](README.md#Index)  
***
### SimpleLayer

In my project, the ORB-SLAM module publishes messages which contain map points of the current frame. In order to make use of those messages and fuse the observations of the laser sensor and the stereo camera, I create a plugin based on the costmap2d package.  
If you are not familiar with the costmap2d package, you may check the [ROS Wiki](http://wiki.ros.org/costmap_2d). Also, there is a [paper](https://ieeexplore.ieee.org/abstract/document/6942636) discuss that.  
In short, the costmap2d is consist of different layers(plugins). Usually, it has three layers, which is static layer, obstacle layer and inflation layer. Those layers works well enough for a ground robot with a laser sensor. However, it would be very complicated to modify the obstacle layer to project the 3D map points to the costmap2d. By contrast, it is much simpler to create a new layer to do the job.  
To help understand the source code, there is a quite simple tutorial on [ROS Wiki](http://wiki.ros.org/costmap_2d/Tutorials/Creating%20a%20New%20Layer), which create a new layer that puts a lethal point on the costmap one meter in front of the robot. Besides, the [source code of the obstacle layer](https://github.com/ros-planning/navigation/blob/melodic-devel/costmap_2d/plugins/obstacle_layer.cpp) may help you as well.  
The SimpleLayer in this module subscribe to the topic `current_mpPoints` and get message which contains a header and a vector.The vector contains map points of the last key frame in ORB-SLAM. Like all subscribers in ROS, there is a callback function to extract infos from messages.  
Another two important functions in the SimpleLayer is `updateBound()` and `updateCost()`. The `updateBound()` function determines the area that the cost map has to update and the `updateCost()` function first determine which cell to be updated and than set the new cost. Note that in the `updateBound()` function, I transform the coordinate first, because the coordinate in Gazebo world and the coordinate in ORB-SLAM is quite different.  
In Gazebo environment, if the orientation of the robot is parallel to the x axis, then the y axis points to the left, and z axis is the height. However, in the stereo camera coordinate, z axis points to the front, x axis points to the right and y is the height. Besides, a rotation matrix and a transition matrix are applied to get the real positions of map points in the Gazebo environment as the robot and the UAV are moving and rotating continuously.  

[Return to Index](README.md#Index)  
***
## Instructions

### Build
**You have to install gmapping and navigation stack before building this module.**  
Install gmapping:  
`sudo apt-get install ros-melodic-gmapping`  
Install navigation stack:  
`sudo apt-get install ros-melodic-navigation`  
I have some dependencies included in the module, but I may still miss some parts since I tried a lot of other packages in the beginning of this project. Therefore, you may follow the error messages of your terminal and install some necessary dependencies.  

**Build Instruction:**  
1. Download two workspaces and put them in your home directory.
2. For drone_ws:    
```
cd ~/drone_ws
catkin build
```
3. For ORBtest_ws:
```
cd ~/ORBtest_ws
catkin_make
```
[Return to Index](README.md#Index)  

### Run

**The ORB-SLAM package needs a roscore to run, so you have to run the rotorS simulator and start simulation first.**  
To run the simulation:  
```
cd ~/drone_ws
roslaunch rotors_gazebo my_mav.launch
```
Run the ORB package:  
```
cd ~/ORBtest_ws
rosrun orb_detector Stereo ./src/orb_detector/Vocabulary/ORBvoc.txt ./src/orb_detector/Examples/Stereo/EuRoC.yaml true
```
If packages are built and run successfully, the result should be like this:  
![Result](https://github.com/QianleLi/EE800/blob/master/images/Result.PNG "Result")   
[Return to Index](README.md#Index)  
### Notes

* Camera parameters  
I used VI-sensor as my stereo camera in this project. If you want to apply the ORB package on other camera models or in real world environment, Remember to modify `EuRoC.yaml` under `~/ORBtest_ws/src/orb_detector/Examples/Stereo`.  
* Communication between the ground robot and the UAV  
As I use ROS and Gazebo to simulate, messages flow through ROS nodes and topics. If you are not familiar with this, please check [ROS Wiki](http://wiki.ros.org/ROS/Tutorials).  

[Return to Index](README.md#Index)  
