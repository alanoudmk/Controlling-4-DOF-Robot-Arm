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

2. Launch RViz with the MoveIt configuration. 
  - MoveIt provides motion planning and visualization capabilities for the robot arm.

```
  roslaunch moveit_pkg demo.launch
``` 

<img src="https://github.com/user-attachments/assets/b3389c08-aa25-4f24-a85d-7227b3d160bd" width="650" height="340">


3. Test
- Click on: 
  > Approx IK Solutions

- Change the Goal State: 
  > Goal State  ->  <rando>

- To start the simulation, click on: 
  > Commands  -> Plane
