# gazebo_sim
This is a ROS package for the Raven II robots using Gazebo.

raven_2arm.launch is the launch file for running. The Gazebo subscribed to the joint_states topic published by r2_control which is the node of controlling the robot.
For r2_control, some codes are commented to ensure it can run without connected to the real robots.
