# Create Robot Arm Package
 on ros 2 foxy



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

5. Examine pakage content:
```
  cd
  cd Robot_Arm_pkg
  ls
  # CMakeLists.txt  include  package.xml  src
```


For [more info](https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html).


***


## 3. URDF Tutorial 

we will be using an alredy built in URDF, but here is som etutorials for more knowaldege:

- [Building a visual robot model from scratch](https://docs.ros.org/en/foxy/Tutorials/Intermediate/URDF/Building-a-Visual-Robot-Model-with-URDF-from-Scratch.html).
- [Learning URDF Step by Step](https://github.com/ros/urdf_tutorial/tree/ros2).



***


## 4. Install dependences (by binary or source):

1. joint-state-publisher


2. joint-state-publisher-gui 


3. urdf-launch






***


## Copy URDF and meshes files from ROS1 package

- [RDF](https://github.com/smart-methods/arduino_robot_arm/blob/main/robot_arm_pkg/urdf/arduino_robot_arm.urdf).

- [meshes](https://github.com/smart-methods/arduino_robot_arm/tree/main/robot_arm_pkg/meshes/stl).






***


## Create launch file


For [more info](https://docs.ros.org/en/foxy/Tutorials/Intermediate/Launch/Creating-Launch-Files.html).






***


## Configure package files and rviz file as needed
