# Create Robot Arm Package
Here are the step-by-step instructions for creating a Robot Arm package in ROS 2 Foxy.

***


## 1. Create ROS 2 Foxy Workspace:

1. Open a new Terminal & Source the ROS 2 Foxy & Create the Workspace Directory::
```
  $ source /opt/ros/foxy/setup.bash
  $ echo $ROS_DISTRO
```
  - The output should be "foxy".
   
```
  $ cd
  $ mkdir ros2_ws
```

2. Create The Source Folder:

  ```
    $ cd ros2_ws/
    $ mkdir src
  ```

3. Install _colcon_:

 ```
   $ sudo apt update
   $ sudo apt install python3-colcon-common-extensions
```

4. Build the Workspace: 

 ```
   $ cd ~/ros2_ws/
   $ colcon build
   $ source install/local_setup.bash
```

5. Source  Workspace: 

 ```
   $ cd ~/ros2_ws/install/
   $ source setup.bash  
  ```

For [more info](https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html).



***


## 2. Create CMake Robot Arm Package for _joint state publisher_ :

1. Navigate to the src foldr:
```
  cd ~/ros2_ws/src
```

2. Create the new package:
```
  ros2 pkg create --build-type ament_cmake  Robot_Arm_pkg
```
  - we will name it _Robot_Arm_pkg_

3. Build the package:
```
  colcon build
```

4. Source the pakage:
```
  source install/local_setup.bash
```

5. Examine the package content:
```
  cd
  cd Robot_Arm_pkg
  ls
  # CMakeLists.txt  include  package.xml  src
```


For [more info](https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html).


***


## 3. URDF Tutorial 

You can use the already built-in URDF, but here are some tutorials for more knowledge:

- [Building a visual robot model from scratch](https://docs.ros.org/en/foxy/Tutorials/Intermediate/URDF/Building-a-Visual-Robot-Model-with-URDF-from-Scratch.html).
- [Learning URDF Step by Step](https://github.com/ros/urdf_tutorial/tree/ros2).



***


## 4. Install dependences:
You can choose to install the dependencies either using binary packages or by building from source, depending on your preference.

1. joint-state-publisher
 - To install package by binary:
   ```
   $ sudo apt-get install ros-foxy-joint-state-publisher
   ```
 - To install package from source:
   ```
   $ cd ~/ros2_ws/src
   $ git clone https://github.com/ros/joint_state_publisher.git
   $ cd ~/ros2_ws
   $ colcon build --packages-select joint_state_publisher
   $ source install/local_setup.bash
   ```



2. joint-state-publisher-gui 
 - To install package by binary:
   ```
   $ sudo apt-get install ros-foxy-joint-state-publisher-gui
   ```
 - To install package from source:
   ```
   $ cd ~/ros2_ws/src
   $ git clone https://github.com/ros/joint_state_publisher.git
   $ cd ~/ros2_ws
   $ colcon build --packages-select joint_state_publisher_gui
   $ source install/local_setup.bash
   ```


3. urdf-launch
 - To install package by binary:
   ```
   $ sudo apt-get install ros-foxy-urdf-launch
   ```
 - To install package from source:
   ```
   $ cd ~/ros2_ws/src
   $ git clone https://github.com/ros/urdf_tutorial.git
   $ cd ~/ros2_ws
   $ colcon build --packages-select urdf_launch
   $ source install/local_setup.bash
   ```






***


## 5. Copy URDF and meshes files from ROS1 package:

1. [URDF](https://github.com/smart-methods/arduino_robot_arm/blob/main/robot_arm_pkg/urdf/arduino_robot_arm.urdf).
 - Download the URDF file from the ROS1 package:
```
 $ cd ~/ros2_ws/src/Robot_Arm_pkg/
 $ mkdir urdf
 $ wget https://raw.githubusercontent.com/smart-methods/arduino_robot_arm/main/robot_arm_pkg/urdf/arduino_robot_arm.urdf -O urdf/arduino_robot_arm.urdf
```


2. [meshes](https://github.com/smart-methods/arduino_robot_arm/tree/main/robot_arm_pkg/meshes/stl).
 - Create the meshes directory and download the mesh files from the ROS1 package:
```
 $ cd ~/ros2_ws/src/Robot_Arm_pkg/
 $ mkdir meshes
 $ cd meshes
 $ wget https://raw.githubusercontent.com/smart-methods/arduino_robot_arm/main/robot_arm_pkg/meshes/stl/base_link.stl
 $ wget https://raw.githubusercontent.com/smart-methods/arduino_robot_arm/main/robot_arm_pkg/meshes/stl/link1.stl
 $ wget https://raw.githubusercontent.com/smart-methods/arduino_robot_arm/main/robot_arm_pkg/meshes/stl/link2.stl
 $ wget https://raw.githubusercontent.com/smart-methods/arduino_robot_arm/main/robot_arm_pkg/meshes/stl/link3.stl
 $ wget https://raw.githubusercontent.com/smart-methods/arduino_robot_arm/main/robot_arm_pkg/meshes/stl/link4.stl
 $ wget https://raw.githubusercontent.com/smart-methods/arduino_robot_arm/main/robot_arm_pkg/meshes/stl/link5.stl
 $ wget https://raw.githubusercontent.com/smart-methods/arduino_robot_arm/main/robot_arm_pkg/meshes/stl/link6.stl
 $ wget https://raw.githubusercontent.com/smart-methods/arduino_robot_arm/main/robot_arm_pkg/meshes/stl/gripper_link.stl
```



***


## 6. Create launch file:

1. Create a new directory:
```
 mkdir launch
```

2. Write the launch file:
 - copy and pate the [File](https://github.com/ros/urdf_tutorial/blob/ros2/launch/display.launch.py).
 - Edit the file as needed to fit your requirements. For example, you may need to update the paths to the URDF and mesh files.

For [more info](https://docs.ros.org/en/foxy/Tutorials/Intermediate/Launch/Creating-Launch-Files.html).






***


## 7. Configure package files and rviz file as needed:

Remember to rebuild your ROS 2 workspace after making changes to the package files or adding new files:

```
 $ cd ~/ros2_ws
 $ colcon build
 $ source install/local_setup.bash
```


1. Edit the Package File:
 - Open the ``package.xml`` file in the Robot_Arm_pkg directory and make any necessary changes, such as adding dependencies or modifying the package information.

2. Edit the RViz File:
 - If you need to customize the RViz visualization, you can create a new RViz configuration file in the Robot_Arm_pkg directory or modify an existing one.

