# Controlling Four Degrees of Freedom Robot Arm 
This tutorial will guide you through the process of controlling a 4-DOF robot arm in ROS Noetic using the MoveIt framework and the Joint State Publisher simulation tool. The goal is to demonstrate how to plan and execute robotic arm movements in a simulated environment.

Enviorment:
  - ROS Distro: Noetic 20.04
  - OS  Version: Ubuntu 20.04.6
  - [Arduino Robot Arm](https://github.com/smart-methods/arduino_robot_arm) Package
  - Using MoveIt and RViz (Joint State Publisher) Simulation
  - Binary build


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

## 1. Create a Workspace

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

## 2. Structure the Robot URDF
For this tutorial, we will be using a pre-written URDF file, so you don't need to create and build it from scratch.

> Here is [How to Structure URDF](https://github.com/alanoudmk/Controlling-4-DOF-Robot-Arm/blob/main/URDF.md).



***

## 3. Install the arduino_robot_arm Package

1. Navigate to the src folder of your workspace:
```
  cd ~/catkin_ws/src
```

2. Install Git if you haven't already:
```
  sudo apt install git
```

3. Clone the arduino_robot_arm repository:
```
  git clone https://github.com/smart-methods/arduino_robot_arm
```

4. Install Dependencies:
```
  cd ~/catkin_ws
  rosdep install --from-paths src --ignore-src -r -y
  sudo apt-get install ros-noetic-moveit
  sudo apt-get install ros-noetic-joint-state-publisher ros-noetic-joint-state-publisher-gui
  sudo apt-get install ros-noetic-gazebo-ros-control joint-state-publisher
  sudo apt-get install ros-noetic-ros-controllers ros-noetic-ros-control
```

5. Compile the catkin workspace:
```
  catkin_make
```

6. Verify Installation:
```
  rospack list
```

- You should see the arduino_robot_arm package listed among the other ROS packages after a successful compilation.





***
# Usage

## Controlling the robot arm by joint_state_publisher

```
  $ roslaunch robot_arm_pkg check_motors.launch
```

Full Instructions [HERE](https://github.com/alanoudmk/Controlling-4-DOF-Robot-Arm/blob/main/joint_state_publisher.md)

## Controlling the robot arm by Moveit and kinematics


```
  $ roslaunch moveit_pkg demo_gazebo.launch
```

Full Instructions [HERE](https://github.com/alanoudmk/Controlling-4-DOF-Robot-Arm/blob/main/MoveIt.md)


***

# Create Your Own Robot Arm Package
[Here](https://github.com/alanoudmk/Controlling-4-DOF-Robot-Arm/blob/main/Create_Robot_Arm_Package.md) are the step-by-step instructions for creating a Robot Arm package in ROS 2 Foxy.
