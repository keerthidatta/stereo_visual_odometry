# Stereo Visual Odometry
include folder contains the source code for computation of visual odometry

## Ros wrapper
src folder contains ros wrapper which converts left and right image topics into opencv image format and calls the vo system 

## Project description
Visual odometry is the technique of tracking robot positions over time using image sequence.
Algorithm has following steps:

1. Left and right image capture
2. Feature detection in left and right images
3. Feature matching between left and rigght images
4. Temporal feature matching
5. Outlier rejection
6. Motion estimation
7. Bundle Adjustment/Optimization

For now, implementations are only done until feature matching. 

Feature detection is perfomed using AKAZE features. 
Feature matching is then performed and unmatched features are removed based on epipolar geometry.

Following are the pre-requisites to run this project in Ubuntu 18.04/16.04

This is ros based project. Images are loaded from the environment through ROS topics. 
But the project can also be run by providing image folder path in the config file.

### opencv4
Install opencv from opencv.org

### ros
Install ros from ros wiki

### Husky
Husky is a robot from clearpath robotics,Inc. 
Mars simulation environment is created and Huksy is loaded in simulated environment. Husky is configured with a stereo camera from which left and right images are captured.

clone husky from https://github.com/keerthidatta/husky_mars.git


### BUILD and INSTALL source
1. cd ~/catkin_ws/src
2. git clone https://github.com/keerthidatta/husky_mars.git
3. git clone https://github.com/keerthidatta/stereo_visual_odometry.git
4. catkin_make
5. source devel/setup.bash

### Usage
* ROS 
In threee different terminals, run

1. roslaunch husky_gazebo_mars_world husky_mars.launch 

2. rosrun stereo_visual_odometry stereo_visual_odom

3. 
![Alt text](img.jpg?raw=true "Control husky")

* Without ROS
1. Provide path to image sequence in the config.yaml file. 
2. Adjust main.cpp as given in example. 
3. In stereo_visual_odometry/include/stereo_visual_odometry/build folder run, ./stereo_visual_odometry ../config/config.yaml. Already configured for few example images

## Rubric points : For UDACITY final project

### compiling and testing
1. used ros cmake_lists and cmake. 
* In stereo_visual_odometry/include/stereo_visual_odometry/build folder : make
* In catkin_ws : catkin_make

### Loops, Functions and I/O
* Loops are used in main.cpp : Line 29
* Functions are used in main.cpp, ros_wrapper, system, tracking
* IO operation used in main.cpp to take calibration file / settings file : Line 17

### Object oriented programming
* Classes created in ros_wrapper, system, tracking, 
* Class constructors utilize member initialization lists. (row_wrapper)
* Classes encapsulate behavior (system, tracking)

### Memory Management
* Makes use of references (system, tracking.cpp)
* Project uses smart pointer : Unique pointer in system.cpp(line 12)
* Project uses move semantics : line 13 sysmtem.cpp

### Concurrency 
* Multithreading will be used in future to run parallel computations