# Controlling the Robot Arm by MoveIt & Kinetmatics 

***

## Setup ROS Enviroment

Before starting, ensure that you have:
- Created the _Catkin_ Workspace
- Downloaded the _arduino_robot_arm_ package


***

## Controlling the movement of the robot by MoveIt & Kinetmatics 

1. Open a New Terminal & Source:
```
  source /opt/ros/noetic/setup.bash
  source ~/catkin_ws/devel/setup.bash
```

2.


***

## RViz with MoveIt:
RViz with MoveIt is used for motion planning and visualization of the robot arm.

```
  roslaunch moveit_pkg demo.launch
``` 

***

## Gazebo with MoveIt:
```
roslaunch moveit_pkg demo_gazebo.launch
```
