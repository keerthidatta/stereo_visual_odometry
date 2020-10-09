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

### opencv4
Install opencv from opencv.org

### ros
Install ros from ros wiki

### Husky
Husky is a robot from clearpath robotics,Inc. 
Mars simulation environment is created and Huksy is loaded in simulated environment. Husky is configured with a stereo camera from which left and right images are captured.

clone husky from https://github.com/keerthidatta/husky_mars.git


### BUILD and INSTALL source
cd catkin_ws/src
git clone https://github.com/keerthidatta/husky_mars.git
git clone https://github.com/keerthidatta/stereo_visual_odometry.git
catkin_make
source devel/setup.bash

### Usage
In threee different terminals, run

1. roslaunch husky_gazebo_mars_world husky_mars.launch 

2. rosrun stereo_visual_odometry stereo_visual_odom

3. ![Alt text](img.jpg?raw=true "Control husky")
