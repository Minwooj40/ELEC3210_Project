# ELEC3210_Project
|    Name     |Student ID|        Email        | Contribution |
|-------------|----------|---------------------|---|
|Minwoo Jung  |20456190  |mjung@connect.ust.hk ||

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
Node: slam
Publish Topic: /slam_out_pose/map/
Subscribe Topic: /initialpose/vrep/scan

We used the [hector_slam](http://wiki.ros.org/hector_slam) package to complete the task. This package helps to generate the map based on the laser scan. It continously publishes its map to `/map`, and alsothe location of the robot which is publised to `/slam_out_pose`. Location of robot will be used in Task 5 for area detection

### Task 2: Control the mobile robot with keyboard
Node: teleop
Publish Topic: /cmd_vel
Subscribe Topic: /vrep/cmd_vel

We used the [teleop_twist_keyboard](http://wiki.ros.org/teleop_twist_keyboard) package to enable controlling via keyboard inputs. The following command line runs the teleop_twist_keyboard and mapping /cmd_vel publish topic to subscribe topic /vrep/cmd_vel
```linux=
$ rosrun teleop_twist_keyboard teleop_twist_keyboard.py /cmd_vel:=/vrep/cmd_vel
```
### Task 3: Image Recognition and Create marker
Node: image_detect
Publish Topic: /vrep/marker
Subscribe Topic: /vrep/image, /vrep/scan, /slam_out_pose

#### Image detection
Step 1: Get the camera image from /vrep/image. The images are flipped before processing.
Step 2: Image detection is performed with [OpenCV's guide about Oriented FAST and Rotated BRIEF + Brute-force descriptor matcher](https://docs.opencv.org/3.4/d1/d89/tutorial_py_orb.html). Prepare descriptors of all given five pictures for future matching with targeted image. 
```python=
self.orb = cv2.ORB_create()
keyPoint, descriptor = self.orb.detectAndCompute(picture, None)
```
Step 3: Get the descriptor for the targeted image and perform BFMatcher to figure ou the best match.
```python=
bf = cv2.BFMatcher(cv2.NORM_HAMMING)
matches = bf.match(des, descriptor)
matches = sorted(matches, key = lambda x:x.distance)
```
#### Localization


### Task 5: Room localization
Node: localization
Publish Topic: None
Subscribe Topic: /slam_out_pose
Step 1: Estimate the coordination for each room A, B, C, D 
Step 2: Depends on the position of the robot measured from the subscribe topic, we can figure out the area that robot places. 
