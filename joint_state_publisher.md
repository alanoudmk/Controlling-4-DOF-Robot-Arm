# Controlling the Robot Arm by ``joint_state_publisher`` Package 


***

## Setup ROS Enviroment

Before Starting Make sure you:
- Created the _Catkin_ Workspace
- Downloaded the _arduino_robot_arm_ package


***

## Control the motors and Visualize them in RViz

1. Open a New Terminal & Source:
```
  source /opt/ros/noetic/setup.bash
  source ~/catkin_ws/devel/setup.bash
```

2. Launch the RViz visualization tool and use the ``joint_state_publisher`` to control the joints of the robot:
```
  roslaunch robot_arm_pkg check_motors.launch
```

- It will Automatically start the ROS Master
![image](https://github.com/user-attachments/assets/6b155b17-0014-467e-9cca-79b6e8594806)


- You should see the robot model displayed in the RViz window:
![image](https://github.com/user-attachments/assets/90fe9493-5fe0-46cd-ba6b-99d4ae63933d)


- The GUI will allow you to control the joint states using sliders to adjust the angles of different joints, and you should see the robot model update in real-time:
![image](https://github.com/user-attachments/assets/c1c61152-4bd7-40b0-a50e-cba3f149f1bf)



***

## Exploring the Node Communication



1. Open a New Terminal & Source:
```
  source /opt/ros/noetic/setup.bash
```

2. visualize the Node Graph:
```
  rqt_graph
```

> If you encounter an ERROR, you can install the necessary packages:
>
>   ```$ sudo apt install ros-noetic-rqt ros-noetic-rqt-graph```


- This will open a window displaying the nodes and topics in your ROS system:




***

To see the actual values being published to the /joint_states topic, use the following command:
```
  rostopic echo /joint_states
```

***

## Launching the Simulation in Gazebo
The following command will launch Gazebo with the robot model and use the joint_state_publisher` to control the joints of the robot in the simulation:
```
  roslaunch robot_arm_pkg check_motors_gazebo.launch
```


***

## Running the Python Script
The Python script joint_states_to_gazebo.py ensures that the joint states from the joint_state_publisher are properly communicated to the Gazebo simulation:
```
  cd ~/catkin_ws/src/arduino_robot_arm/robot_arm_pkg/scripts
  sudo chmod +x joint_states_to_gazebo.py
  rosrun robot_arm_pkg joint_states_to_gazebo.py
```
