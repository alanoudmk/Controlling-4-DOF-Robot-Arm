# Controlling the Robot Arm by ``joint_state_publisher`` Package 


***

## Setup ROS Enviroment

Before starting, ensure that you have:
- Created the _Catkin_ Workspace
- Downloaded the _arduino_robot_arm_ package


***

## Controlling the Motors and Visualizing them in RViz

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

    <img src="https://github.com/user-attachments/assets/6b155b17-0014-467e-9cca-79b6e8594806" width="390" height="260">


- You should see the robot model displayed in the RViz window:

    <img src="https://github.com/user-attachments/assets/90fe9493-5fe0-46cd-ba6b-99d4ae63933d" width="390" height="200">


- The GUI will allow you to control the joint states using sliders to adjust the angles of different joints, and you should see the robot model update in real-time:
  
    <img src="https://github.com/user-attachments/assets/c1c61152-4bd7-40b0-a50e-cba3f149f1bf" width="150" height="220">



***

### Exploring the Node Communication

1. Open a New Terminal & Source:
```
  source /opt/ros/noetic/setup.bash
```

2. visualize the Node Graph:
```
  rqt_graph
```

> If you encounter an ERROR, you can install the necessary packages:

  ```
   $ sudo apt install ros-noetic-rqt ros-noetic-rqt-graph
   ```

   <img src="https://github.com/user-attachments/assets/b89a6a92-23bd-445f-b6dc-34496f3afb4d" width="390" height="220">


- `` /joint_state_publlsher_gul`` :  Node representing the GUI window
- ``/robot_state_publlsher`` :Node representing the Robot Arm
- `` /joint_sates`` :  Topic that connects the two nodes



***

### Printing Topic Actual Values

1. Open a New Terminal & Source:
    ```
   source /opt/ros/noetic/setup.bash
    ```

2. Display the actual values being published on the /joint_states topic:

    ```
    rostopic echo /joint_states
    ```

     <img src="https://github.com/user-attachments/assets/8465197c-5ed5-4082-b7e5-014c753ea81f" width="350" height="200">



3. Now, move the robot arm by adjusting the angles of the joints using the GUI:

   <img src="https://github.com/user-attachments/assets/cf51a79b-d0dc-44b3-8407-6356ace51a70" width="350" height="200">
   <img src="https://github.com/user-attachments/assets/32b7923a-d280-4b1e-a1b5-2a04bba49b87" width="130" height="200">


4. Click _Center_ to return the robot arm to its initial position:

   <img src="https://github.com/user-attachments/assets/776e61e5-8964-45a2-a2bb-ea7361dfad39" width="350" height="200">
   <img src="https://github.com/user-attachments/assets/8bf8c20d-05d9-459c-8624-a91e998bcf6d" width="130" height="200">


***

# Launching the Simulation in Gazebo

To launch the Gazebo simulation with the Robot Arm Model and control the robot's joints, run:

```
  roslaunch robot_arm_pkg check_motors_gazebo.launch
```
 - This command will start the Gazebo simulation with the _Robot Arm Model_ and enable the _joint_state_publisher_ to control the robot's joints.

***

## Establishing Communication between Gazebo and the Robot Arm

To ensure proper communication of the joint states between the joint_state_publisher and the Gazebo simulation, you may need to make the necessary **Permissions* changes by running a Python script.


The Python script joint_states_to_gazebo.py ensures that the joint states from the joint_state_publisher are properly communicated to the Gazebo simulation:

1. Navigate to the scripts folder :
```
  cd ~/catkin_ws/src/arduino_robot_arm/robot_arm_pkg/scripts
```

2. Make it executable:

```
  sudo chmod +x joint_states_to_gazebo.py
```

3. Launch the script:
```
  rosrun robot_arm_pkg joint_states_to_gazebo.py
```


- This Python script, joint_states_to_gazebo.py, ensures that the joint states from the joint_state_publisher are properly communicated to the Gazebo simulation, allowing for seamless control of the robot arm's joints.
