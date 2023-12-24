# Cobot


![App Screenshot](https://github.com/yuvimehta/Cobot/blob/main/Screenshots/Screenshot%20from%202023-12-24%2002-08-44.png)
## Overview

Hey there,
This project is designed to provide a hands-on learning experience for beginners interested in robotics, ROS, and robot kinematics. The focus is on simulating a 5-DOF robotic arm in Gazebo, with easy-to-understand simulation files, basic control scripts, a custom GUI for control, and simple Inverse Kinematics (IK) solvers.



## Getting Started

### Prerequisites

- ROS (Robot Operating System)
- Gazebo
- Python (for control scripts and GUI)







### Installation

1. First you need to make a ros workspace. If you are new to ros check out this installations manual. [ROS Installation](https://wiki.ros.org/ROS/Installation)

2. Create a ros workspace and clone this repo in src folder

```bash
  mkdir ros_ws
  cd ros_ws
  mkdir src
  cd ..
  catkin_make
```
3. Clone this repo in the src and then build the package 

```bash
cd src
git clone -b master https://github.com/yuvimehta/cobot.git

cd ..
catkin_make
```


### Test Run

##### After building the cloned package source the workspace using this command
```bash
source ~/ros_ws/devel/setup.bash
```
##### To simply display bot in gazebo
```bash
roslaunch cobot_description display.launch
```
##### Running gazebo with working model
```bash
roslaunch cobot_controller gazebo.launch
rosrun cobot_controller test.py
```
##### Open another terminal and run to get slider gui
```bash
roslaunch cobot_gui gui.launch
```
Now cobot is all set to work.

![App Screenshot](https://github.com/yuvimehta/Cobot/blob/main/Screenshots/Screenshot%20from%202023-12-24%2002-10-20.png)


Make different gazebo worlds for usecases,try out different control methods by integrating different tech like voice commands, computer vision etc..

## Planned add-ons 
1. Adding custom Simple IK/FK solver 
2. Enhancing GUI for path planning and other features
3. Adding other functionalties like rviz,rqt 
4. planning to integrate grippers for usecases