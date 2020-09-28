# Online Transfer Learning for 3D LiDAR-based Human Detection using YOLO human tracker
[![License](http://img.shields.io/:license-mit-blue.svg?style=flat-square)](http://badges.mit-license.org) 

This repository is part of the Thesis project for the Robotics and Autonomous Systems MSc in Lincoln University UK. The project is a ROS-based online transfer learning framework for human classification in 3D LiDAR scans, utilising a YOLO-based tracking system. The motivation behind this is that the size of the training data needed to train such classifiers should be considerably large. In addition annotating said data is a tedious work with high costs.

![shot](https://user-images.githubusercontent.com/41471360/94432009-c5355780-019e-11eb-8952-7f1d9f41421a.png)

The main contributions of this work are:
  - Development and implementation of a YOLO tracker from previous work.
    The tracker produced a linear angle approximation between each person and the robot.
  - The integration of the YOLO detector to the Bayesian tracking system 
    (Work presented in [Online learning for human classification in 3D LiDAR-based tracking](https://github.com/yzrobot/online_learning))
  -  The evaluation of the new tracker and the trained classifier. 

## System architecture
![system](https://user-images.githubusercontent.com/41471360/94431472-024d1a00-019e-11eb-80e8-28e691ffec1d.png)

## Tech
In order to develop this code the following were used:

* [ROS](https://www.ros.org/)
* [YOLO: Real-Time Object Detection](https://github.com/leggedrobotics/darknet_ros)
* [Online learning for human classification in 3D LiDAR-based tracking](https://github.com/yzrobot/online_learning)

## Installation

Assuming ROS is installed in the system (if not follow [instructions](http://wiki.ros.org/kinetic/Installation))

```bash
// Install prerequisite packages 
$ sudo apt-get install ros-kinetic-velodyne*
$ sudo apt-get install ros-kinetic-bfl
$ cd catkin_ws/src
```
For running the other trackers install the ros packages from [here](https://github.com/LCAS/rosdistro/wiki)
```
// The core 
$ git clone --recursive https://github.com/parisChatz/yolo-online-learning.git
// Build
$ cd catkin_ws
$ catkin_make -DCMAKE_BUILD_TYPE=Release
```
## Testing and executing
In order to test the Yolo tracker with a simple rosbag (with camera topic) first be sure to change the robot camera topic if needed in
* /darknet_ros/darknet_ros/config/ros.yaml
* /people_yolo_angle_package/scripts/people_position.py

Then do:
```
$ roslaunch people_yolo_angle_package testing.launch bag:="/dir_to_rosbag/example_rosbag.bag"
```
For executing the whole code:
```
roslaunch people_yolo_angle_package object3d_yolo.launch 
```
This will open the rosbag mentioned in the launchfile as well as the whole system environment including RVIZ.

## Results
The results showed decreased performance in the case of the YOLO detector. 
When YOLO was paired with other detectors the performance seems to increase. 

![300_iter_new](https://user-images.githubusercontent.com/41471360/94431526-155fea00-019e-11eb-9ea1-caae9fc93988.png)

It was also shown that the shortcomings of
the YOLO tracker do not solemnly lie in the human
detection.
![iou](https://user-images.githubusercontent.com/41471360/94431531-1729ad80-019e-11eb-9c3d-c36f18c7968e.png)

A comparison was made of the system that uses data from the Yolo tracker and ground truth data which was inputed in the system in the same way as Yolo produces data.
The results are the following. 

![g_truth](https://user-images.githubusercontent.com/41471360/94432336-4e4c8e80-019f-11eb-9e74-7079ee807682.png)

The evaluation on the bounding boxes of
YOLO showed that YOLO detects bigger bounding
boxes for moving people but the weakest point of
the system is rather in the linear approximation of
the angle that was calculated.

### Future work
* Improvement of the angle approximation of Yolo tracker
* Training/testing the system with bigger data
