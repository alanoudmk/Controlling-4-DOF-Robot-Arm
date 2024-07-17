# Controlling Four Degrees of Freedom Robot Arm 
This tutorial will guide you through the process of controlling a 4-DOF robot arm in ROS Noetic using the MoveIt framework and the Joint State Publisher simulation tool. The goal is to demonstrate how to plan and execute robotic arm movements in a simulated environment.

We will be working on:
  - ROS Noetic 20.04
  - [Arduino Robot Arm](https://github.com/smart-methods/arduino_robot_arm) Package
  - Using MoveIt and Joint State Publisher Simulation


***

## Robot Arm
The robot arm has 5 joints only 4 joints can be fully controlled via ROS and Rviz, the last joint (gripper) has a default motion executed from the Arduino code directly.

The robot arm has 5 joints in total.
Only 4 of the 5 joints can be fully controlled via ROS and Rviz (the visualization tool).
The last joint, which is the gripper, has a default motion executed directly from the Arduino code.


Robot initial positions:

<img src="https://github.com/user-attachments/assets/52a06f9f-15c9-4fee-aa1f-b1694d5fbb9b" width="390" height="280">


***

# Preparing ROS

## Create a Workspace:

1. Source the ROS Noetic setup file:
```
  $ source /opt/ros/noetic/setup.bash
  $ echo $ROS_DISTRO
```
  - The output should be "noetic".
    
2. Create and build a Catkin workspace:
```
  mkdir -p ~/catkin_ws/src
  cd ~/catkin_ws/
  catkin_make
```

3. Source the workspace:
```
  $ source devel/setup.bash
  $ cd devel/
  $ source setup.bash
```

4. Add the workspace to your .bashrc to source it automatically:
```
  echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
```

> For more information, please refer to the [ROS Wiki](http://wiki.ros.org/catkin/Tutorials/create_a_workspace).

***

## Structuring the robot URDF:
For this tutorial, we will be using a pre-written URDF file, so you don't need to create and build it from scratch.

> Here is [How to Structure URDF](https://github.com/alanoudmk/Controlling-4-DOF-Robot-Arm/blob/main/URDF.md).



***

# Usage

## Controlling the robot arm by joint_state_publisher

## Controlling the robot arm by Moveit and kinematics
