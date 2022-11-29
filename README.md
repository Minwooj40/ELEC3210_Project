# ELEC3210_Project
|    Name     |Student ID|        Email        | Contribution |
|-------------|----------|---------------------|---|
|Minwoo Jung  |20456190  |mjung@connect.ust.hk ||
|Jaechul Roh  |20473590  |jroh@connect.ust.hk ||

## Introduction
This project aims to implement a mobile robot operating in a simulation environment with ROS and CoppeliaSim. The robot operation is designed to be controlled by keyboard input and its laser can data can construct the a map of environment which allows the robot's position determination. Moreover, the mobile robot detects face images using camera sensors and predict its position at the same time. Lastly, the robot follows the path of yellow ball. These process can be switched to automatic mode as well. 

## Environment setup
### Environments used
* Ubuntu 20.04
* ROS: Noetic
* V-REP 3.6.2
* rviz 1.14.10
* OpenCV 4.2.0
### Additional packages
* teleop_twist_keyboard 1.0.0
* hector_mapping 0.5.2

## Methodology
### Task 1: Build 2D grid map with laserscan data via rviz
Node: hector_mapping
Publish Topic: /slam_out_pose/map/
Subscribe Topic: /initialpose/vrep/scan

We used the [hector_slam](http://wiki.ros.org/hector_slam) package to complete the task. This package generates the environment map and publishes to `/map`. It also produces information about the location of the robot which is publised to `/slam_out_pose`. This will be used for later steps. 

### Task 2: Control the mobile robot with keyboard
Node: teleop
Publish Topic: /cmd_vel
Subscribe Topic: /vrep/cmd_vel

We used the [teleop_twist_keyboard](http://wiki.ros.org/teleop_twist_keyboard) package to enable controlling via keyboard inputs. The following command line runs the teleop_twist_keyboard and mapping /cmd_vel publish topic to subscribe topic /vrep/cmd_vel
```linux=
$ rosrun teleop_twist_keyboard teleop_twist_keyboard.py /cmd_vel:=/vrep/cmd_vel
```
