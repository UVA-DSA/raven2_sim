#!/usr/bin/env python
"""
Subscribes to joint_states, publishes joint commands to update gazebo sim.
"""

__author__ = "Yongming Qin"
__version__ = "0.0.0"
__status__ = "Prototype"
__date__ = "2018/11/15"

import rospy
import numpy as np
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Quaternion, Vector3
from std_msgs.msg import Float64
import tf
from math import sin, cos

pub_shoulder_L      = rospy.Publisher('/raven_2arm/shoulder_L_position_controller/command',
                                        Float64, queue_size=10)
pub_elbow_L         = rospy.Publisher('/raven_2arm/elbow_L_position_controller/command',
                                        Float64, queue_size=10)
pub_insertion_L     = rospy.Publisher('/raven_2arm/insertion_L_position_controller/command',
                                        Float64, queue_size=10)
pub_tool_roll_L     = rospy.Publisher('/raven_2arm/tool_roll_L_position_controller/command',
                                        Float64, queue_size=10)
pub_wrist_joint_L   = rospy.Publisher('/raven_2arm/wrist_joint_L_position_controller/command',
                                        Float64, queue_size=10)
pub_grasper_1_L     = rospy.Publisher('/raven_2arm/grasper_joint_1_L_position_controller/command',
                                        Float64, queue_size=10)
pub_grasper_2_L     = rospy.Publisher('/raven_2arm/grasper_joint_2_L_position_controller/command',
                                        Float64, queue_size=10)

pub_shoulder_R      = rospy.Publisher('/raven_2arm/shoulder_R_position_controller/command',
                                        Float64, queue_size=10)
pub_elbow_R         = rospy.Publisher('/raven_2arm/elbow_R_position_controller/command',
                                        Float64, queue_size=10)
pub_insertion_R     = rospy.Publisher('/raven_2arm/insertion_R_position_controller/command',
                                        Float64, queue_size=10)
pub_tool_roll_R     = rospy.Publisher('/raven_2arm/tool_roll_R_position_controller/command',
                                        Float64, queue_size=10)
pub_wrist_joint_R   = rospy.Publisher('/raven_2arm/wrist_joint_R_position_controller/command',
                                        Float64, queue_size=10)
pub_grasper_1_R     = rospy.Publisher('/raven_2arm/grasper_joint_1_R_position_controller/command',
                                        Float64, queue_size=10)
pub_grasper_2_R     = rospy.Publisher('/raven_2arm/grasper_joint_2_R_position_controller/command',
                                        Float64, queue_size=10)


def callback(data):
    # publish separate joint commands to each *command topic
    pub_shoulder_L.publish(Float64(data.position[0]))
    pub_elbow_L.publish(Float64(data.position[1]))
    pub_insertion_L.publish(Float64(data.position[2]))
    pub_tool_roll_L.publish(Float64(data.position[3]))
    pub_wrist_joint_L.publish(Float64(data.position[4]))
    pub_grasper_1_L.publish(Float64(data.position[5]))
    pub_grasper_2_L.publish(Float64(data.position[6]))

    pub_shoulder_R.publish(Float64(data.position[7]))
    pub_elbow_R.publish(Float64(data.position[8]))
    pub_insertion_R.publish(Float64(data.position[9]))
    pub_tool_roll_R.publish(Float64(data.position[10]))
    pub_wrist_joint_R.publish(Float64(data.position[11]))
    pub_grasper_1_R.publish(Float64(data.position[12]))
    pub_grasper_2_R.publish(Float64(data.position[13]))

if __name__ == '__main__':
    # Recieves joint_states msgs, calls talker to update raven sim joint positions
    rospy.init_node('msg_bridge_abs', anonymous=True)
    rospy.Subscriber("/joint_states", JointState, callback)
    rospy.spin()
    

