# Controlling the Robot Arm by ``joint_state_publisher`` Package 

## Control the motors and Visualize them in RViz

The following command will launch the RViz visualization tool and use the joint_state_publisher to control the joints of the robot:
```
  roslaunch robot_arm_pkg check_motors.launch
```




***

The GUI allows you to control the joint states using sliders to adjust the angles of different joints, and you should see the robot model update in real-time:





***

To visualize the node graph and understand the communication between nodes, use the following command:
```
  rqt_graph
```
This will open a window showing the nodes and topics in your ROS system:




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
