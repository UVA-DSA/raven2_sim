======================================
Summary and licensing information
======================================
This respitory contains the code for RAVEN II surgical simulator. It is based on the open-source RAVEN II control software, developed by the University of Washington Biorobotics lab at: https://github.com/uw-biorobotics/raven2 and the first version of simulator developed by Homa Alemzadeh, Daniel Chen, and Xiao Li, at the University of Illinois at: https://github.com/CSLDepend/raven2_sim.

It enables running RAVEN control software with no robotic hardware attached. A Python script (Real_Packet_Generator_Surgeon.py) mimicks the network packets sent from the surgeon console based on a previously collected data from the trajectory of a basic surgical task. ROS  visualization packages (rviz and Gazebo) are used for 3D animation of the robotic motions and the interactions with environment. Different modes enable the simulator to work with or without the surgeon console or in parallel with the actual robot. The fault injection mode enables testing the resilience of the robot to accidental faults and malicious attacks by artificially inserting faults into the control software. 
For more information, see the following papers: 
http://web.engr.illinois.edu/~alemzad1/papers/MedicalCPS_2015.pdf
http://arxiv.org/abs/1504.07135v1
http://faculty.virginia.edu/alemzadeh/papers/DSN_2016.pdf 

Copyright (C) 2018 Dependable Systems and Analytics Group, University of Virginia

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.

======================================
How to run
======================================
Setting up the repository:
0. Clone the repository:
   "git clone https://github.com/UVA-DSA/raven2_sim.git"
1. Rename the folder name from "raven2_sim" to "raven_2"
2. Change the ROS_PACKAGE_PATH environment variable to the location of raven_2 folder. For example:
   "export ROS_PACKAGE_PATH=/home/raven/raven_2:/home/raven/raven_2/raven_visualization:/opt/ros/kinetic/share:/opt/ros/kinetic/stacks"
   To test if the change was made successfully, run "roscd raven_2" and you should be relocated to the raven_2 folder.
4. Run "tar zxvf ./teleop_data/new_test_data.tgz -C ./teleop_data/" to unzip the datafiles used by the packet generator

Running RAVEN simulator using trajectory 2:
1. Goto raven_2 folder:  "roscd raven_2"
2. Simple  simulator /w packet-gen: "python run.py sim 1 none traj2"
   Dynamic simulator /w packet-gen: "python run.py dyn_sim 1 none traj2"
   Robot /w packet-gen:  "python run.py rob 1 none traj2"
   Robot /w surgeon-gui: "python run.py rob 0 none traj2" 

Branches:
- simulator:  Simple simulator with no fault injection capabilities
- gazebo_Sim: Simulator integrated with Gazebo and fault injection capabilities
- rviz_sim:   Simulator with rviz and fault injection capabilities
- raven_auto: rviz_sim with motion planner simulating autonomous debridment within a virtual dome
