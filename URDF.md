# What is URDF?


URDF (Unified Robot Description Format) is:
- an XML-based file format.
- used to describe the physical and visual characteristics of a robot.
- Radians (rad) used to specify the angular positions and rotations of robot links and joints.
- Meters (m)  used to specify the linear dimensions and positions of robot links and joints.


**

## URDF Components

Links: - Represent the rigid bodies of the robot. - Each link can have visual, collision, and inertial properties.
Joints: - Define the relationship between two links. - Can specify different types of joints, such as fixed, revolute, prismatic, and continuous.
Visuals: - Describe how the link should be displayed in simulations and visualization tools.
Collisions: - Define the collision properties for physical simulations and interactions.
Inertials: - Specify the mass and inertia properties for dynamic simulations.
Sensors and Actuators: - Describe the sensors and actuators attached to the robot.
The URDF format allows for the creation of detailed and accurate robot models that can be used in various ROS applications, such as simulations, visualizations, and motion planning.




The key points for structuring a robot's URDF (Unified Robot Description Format) are:

- Robot Element: The top-level <robot> element contains the entire description of the robot.
- Link Elements: Each physical component of the robot is described using a <link> element, which specifies the visual, collision, and inertial properties of that component.
- Joint Elements: The connections between the robot's links are defined using <joint> elements, which specify the type of joint (e.g. revolute, prismatic) and the parameters that govern its motion.
- Transmission Elements: If the robot has actuated joints, <transmission> elements are used to describe the relationship between the actuator and the joint.
- Gazebo Extensions: If the robot will be simulated in Gazebo, additional <gazebo> elements can be used to specify Gazebo-specific properties.


***
# How to Create UEDF?


## Manually create a URDF 

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
