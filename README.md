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

