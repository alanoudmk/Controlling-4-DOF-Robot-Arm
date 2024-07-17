# What is URDF?


URDF (Unified Robot Description Format) is:
- an XML-based file format.
- used to describe the physical and visual characteristics of a robot.
- Radians (rad) used to specify the angular positions and rotations of robot links and joints.
- Meters (m)  used to specify the linear dimensions and positions of robot links and joints.


***

## URDF Components
1. Robot:
   - The root element of the URDF file, which defines the name of the robot.
2. Link:
   - Represents a rigid body part of the robot, such as a base, arm, or wheel. Each link has visual, collision, and inertial properties.
3. Joint:
   - Defines the connection between two links, specifying the type of joint (e.g., revolute, prismatic, fixed), the axis of rotation/translation, and the limits of the joint's motion.
4. Transmission: 
   - Describes the relationship between the joint and the actuator that drives it, such as the gear ratio and mechanical reduction.
5. Material:
    - Defines the visual materials (e.g., colors, textures) assigned to the robot's links.
6. Mimic:
    - Allows one joint to mimic the motion of another joint, useful for coordinating the motion of multiple joints.
7. Dynamics:
    - Defines the dynamic properties of the links, such as mass, inertia, and friction coefficients
8. Gazebo:
    - Specifies the plugins and sensor information required for simulation in the Gazebo environment.
9. Plugin:
    - Allows the integration of custom functionality, such as controllers, sensors, and other extensions.

  <img src="https://github.com/user-attachments/assets/4a3ec6b2-a217-439f-84fb-117a729aea3a" width="200" height="250">
   <img src="https://github.com/user-attachments/assets/03c6043d-8118-4c56-98a6-186cc7a57d4b" width="200" height="250">



***
# How to Create UEDF?


## Manually  

Example of how to manually create a URDF (Unified Robot Description Format) file for a simple robot model. URDF is an XML-based format used to describe the components and kinematics of a robot.

  <img src="https://github.com/user-attachments/assets/0f9e136f-31cd-43b7-bb31-9190ef1c26aa" width="200" height="250">


***

## Gasipo Simulation
Gasipo is a Python-based tool for generating Unified Robot Description Format (URDF) files from a simplified description format.

1. Install gasipo and ROS dependencies
2. Create a gasipo configuration file
3. Generate the URDF file
4. Integrate the URDF file into your ROS workspace:
5. Create a new ROS package or add the URDF file to an existing one.
6. Launch the robot in Gazebo.
7. Run the simulation.

***

## Blender

Blender is a open-source 3D creation suite that is widely used in the fields of visual effects, animation, and 3D modeling. powerful tool that allows users to create 3D models, animate them, and even produce rendered images and videos. It has a wide range of features and capabilities, making it a versatile tool for various creative and technical applications.

Here's a general outline of the process:
  1. Set up Blender: Open Blender and create a new scene.
  2. Model the Robot: Use Blender's modeling tools to create the visual representation of your robot. This includes the robot's links (individual parts) and joints.
  3. Add Collision Geometry: Ensure that each link has a collision geometry associated with it. This is important for accurate physics simulation and collision detection.
  4. Define Joints: Create the joints that connect the robot's links. Specify the joint type (e.g., revolute, prismatic, fixed) and the joint's axis of rotation or translation.
  5. Assign Coordinate Frames: Assign coordinate frames to each link and joint to define the robot's kinematic structure.
  6. Export to URDF: Once the robot model is complete, export the scene to a URDF file.

> Step by Step: [Structuring the Robot by Using Blender](https://drive.google.com/file/d/13Zwar6_7NBtJJoh1Uhmcws74_DmFbx__/view)
>
> [From Blender to URDF with Phobos](https://www.youtube.com/watch?v=JGPyNxzVlYA&t=222s)
