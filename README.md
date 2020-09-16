# RoboMuse-4.0

## ROS Package for IIT Delhi's Indigenous Mobile Manipulator - RoboMuse 4.0

RoboMuse is a mobile robot platform. Its development started in 2009 and was initiated by Prof. SK Saha. It started as a line following robot, and over the years, it has had many iterations, with each iteration adding something unique to the robot. The main aim of RoboMuse was to be able to function without any human interaction. This was achieved on RoboMuse 4.0, using the Robot Operating System (ROS). RoboMuse 4.0 was later converted into a mobile manipulator by mounting a 5-DOF manipulator and was used to perform a pick-and-place task using various computer vision techniques. This repository contains the ROS package files for RoboMuse 4.0.

### This package includes files for: 

1) Simulation of the robot in Gazebo Physics Simulator. 

2) Computer vision algorithms to detect predefined object in the environment using SIFT feature matching based object detection.

3) Inverse Kinematics solvers to establish the cartesian control of the 5-DOF manipulator. 

4) Hardware drivers were implemented for cartesian-based control and trajectory control of the manipulator to exploit its full capability.

5) Implementation of techniques to perform a single complex task of grasping an object from the environment and placing it at the desired location. 

6) Visual odometry generation package based on Rtabmap-ROS

7) Wheel odometry and visual odometry fusion using an extended Kalman filter. 

8) Multi-robot systems simulations using ROS and Gazebo. 

9) Simulation system to provide real world environment inputs to the virtual robot in simulation by using webcams. 

10) Autonomous human feeding robot system that is able to recognize gesture-based commands using visual inputs.
