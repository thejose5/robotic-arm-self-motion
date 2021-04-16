# Robotic Arm Self Motion

This project was a personal learning project I worked on to develop an understanding of ROS development using C++, Gazebo and Moveit. The goal of the project was to enable self motion in a robotic arm. 

**Self Motion:**
Self motion refers to moving the robotic arm without changing the location of the end effector. 

## How the system works:
The joint velocities required for self motion are calculated by the equation qdot = (J * pinv(J) - I7)*Z, where J is the jacobian of the arm at its current position, pinv(J) is the pseudoinverse of the Jacobian, Z is a 7x1 vector of ones and I7 is a 7x7 identity matrix. 

The ROS msg used to communicate with the robot controller is the joint_trajectory msg which takes in position and velocity at various time steps. As a result, the joint_trajectory message is created by using the joint velocities found earlier to calculate the position of each joint after each timestep. 

Although this is approach to self motion does not result in a perfect solution, since this was a learning project, I decided that this would suffice.  


