#!/usr/bin/env python
""" msg_converter.py
Subscribes to joint_states, publishes joint commands to update gazebo sim.
"""
from __future__ import print_function

__author__ = "Yongming Qin"
__version__ = "0.0.0"
__status__ = "Prototype"
__date__ = "2018/11/17"

import rospy
import numpy as np
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Quaternion, Vector3
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import tf
from math import sin, cos, pi

pub_joint_abs = rospy.Publisher('/raven_2arm/set_joint_trajectory', JointTrajectory, queue_size=10)

# Recieves joint_states msgs, calls callback to update raven sim joint positions
def callback(data):
    #print("in callback")
    traj_msg.header.stamp = rospy.Time.now()
    #traj_msg.header.seq = 
    traj_msg.header.frame_id = 'world'
    
    jpos_rad = list(data.position[0:14])
    # append right arm's homing position
    #jpos_rad.extend([0.5227462649345398, 1.5772970914840698, -0.06990847110748288, 0.710763909418719,
                        #0.05527101457118988, -0.021630939096212387, 0.021630939096212387])

    #print(jpos_rad)
    #jpos_rad = [0.5219423770904541, 1.5778145790100098, -0.0695327830314636, -0.704855569203036, 0.047629814594984055, -0.005959832109510899, 0.005959832109510899,
        # 0.5227462649345398, 1.5772970914840698, -0.06990847110748288, 0.710763909418719]

    traj_pt = JointTrajectoryPoint(positions=jpos_rad, time_from_start=rospy.Duration(0.0))
    
    traj_msg.points = [traj_pt]
    


rospy.init_node('msg_bridge_pid', anonymous=True)
rospy.Subscriber("/joint_states", JointState, callback)


traj_msg = JointTrajectory(joint_names=['shoulder_L', 'elbow_L', 'insertion_L', 'tool_roll_L',
                                'wrist_joint_L', 'grasper_joint_1_L', 'grasper_joint_2_L',
                                 'shoulder_R', 'elbow_R', 'insertion_R', 'tool_roll_R',
                                'wrist_joint_R', 'grasper_joint_1_R', 'grasper_joint_2_R'])


def bridge():
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        pub_joint_abs.publish(traj_msg)
        #print(traj_msg.points)
        rate.sleep



def test():
    traj_msg.header.stamp = rospy.Time.now()
    #traj_msg.header.seq = 
    traj_msg.header.frame_id = 'world'

    # right grasper negative
    # [0.5219423770904541, 1.5778145790100098, -0.0695327830314636, -0.704855569203036, 0.047629814594984055, -0.005959832109510899, 0.005959832109510899, 
        # 0.5227462649345398, 1.5772970914840698, -0.06990847110748288, 0.710763909418719]
    jpos_deg = [29.9, 90.4, 22.94, 4.61, 2.72, 45.7, 46.39,
                                29.9, 90.37, 22.92, -4.27, -3.16, 48.0, 45.5]
    jpos_rad = ( np.array(jpos_deg) / 180*pi ).tolist()

    traj_pt = JointTrajectoryPoint(positions=jpos_rad, time_from_start=rospy.Duration(0.0))
    

    traj_msg.points = [traj_pt]
    
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        pub_joint_abs.publish(traj_msg)
        #print(traj_pt)
        rate.sleep



if __name__ == '__main__':
    
    
    #test()
    bridge()




    
    
