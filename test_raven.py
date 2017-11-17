import argparse
parser = argparse.ArgumentParser()
parser.add_argument("--interactive", action="store_true")
args = parser.parse_args()

import openravepy
import trajoptpy
import json
import time
import math
import csv
import numpy as np
import rospy
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import MultiArrayDimension
pubCount = 0
n_steps = 1500
env = openravepy.Environment()
env.StopSimulation()
env.Load('/home/uva-dsa1/raven_2/src/raven/trajopt/data/raven_with_workspace.zae')
env.Load('/home/uva-dsa1/raven_2/src/raven/trajopt/data/table.xml')


trajoptpy.SetInteractive(args.interactive) # pause every iteration, until you press 'p'. Press escape to disable further plotting
robot = env.GetRobots()[0]
print robot
print robot.GetManipulators()[1].GetEndEffectorTransform()#[0].GetArmIndices()
joint_start = []
joint_target = []
#joint_start = [29.9499568939,	90.4525146484,	22.9035243988,	11.5578584671,	7.2847337723,	82.4116973877,	56.6583213806]
#joint_start = [30, 90,	22.9183120728,	245.224884033,	86.7720489502,	101.590637207]
joint_start = [0,0, 0,0,0,0]

#oint_start = [29.9506454468,	90.4491882324,	22.9347839355,		8.3686666489,	5.7021679878,	43.5009040833]


#joint_target = [27.1548213959,	88.848487854,	22.617143631,	6.5820627213,	0.1315095425,	64.2381439209]
#joint_target = [26.903547287,	89.645614624,	22.6126232147,	-10.7246627808,	0.4170058668,	65.9799118042]
#joint_target = [27.1548213959,	88.848487854,	22.617143631,	6.5820627213,	0.1315095425,	64.2381439209,	4.1166629791]
joint_target = [31.8029937744,	88.3655548096,	22.5955314636,	-7.0923576355,	23.0211296082,	66.2951507568]

joint_start_1 = [27.1548213959,	88.848487854,	22.617143631,	6.5820627213,	0.1315095425,	64.2381439209]
joint_target_1 = [31.8029937744,	88.3655548096,	22.5955314636,	-7.0923576355,	23.0211296082,	66.2951507568]

for i in range (len(joint_target)):
	joint_target[i]= math.radians(joint_target[i])
	joint_start[i]= math.radians(joint_start[i])
#print joint_start

joint_start[2]= joint_start[2]/1000
joint_target[2]= joint_target[2]/1000

print joint_target

robot.SetDOFValues(joint_start, robot.GetManipulators()[1].GetArmIndices())



request = {
"basic_info" : {
	"n_steps" : n_steps,
	"manip" : "right_arm", #might want to change this value
	"start_fixed" : True
	},
"costs" : [
	    {
	        "type" : "joint_vel",
	        "params": {"coeffs" : [1]}
	        },
	    {
	        "type" : "collision",
	        "params" : {
	            "coeffs" : [100],
	            "continuous" : True,
	            "dist_pen" : [0.001]
	            }
	        },
	   {
	    "type" : "collision",
	    "params" : {
	        "coeffs" : [100],
	        "continuous" : False,
	        "dist_pen" : [0.001]
	        }
	    },
	    ],
"constraints" : [
	{
	    "type" : "joint", # joint-space target
	    "params" : {"vals" : joint_target } # length of vals = # dofs of manip
	    }
	    
	],
"init_info" : {
	"type" : "straight_line",
	"endpoint" : joint_target
	}
}

s = json.dumps(request) # convert dictionary into json-formatted string
prob = trajoptpy.ConstructProblem(s, env) # create object that stores optimization problem
t_start = time.time()
result = trajoptpy.OptimizeProblem(prob) # do optimization
t_elapsed = time.time() - t_start

print result
print "optimization took %.3f seconds"%t_elapsed

from trajoptpy.check_traj import traj_is_safe
prob.SetRobotActiveDOFs() # set robot DOFs to DOFs in optimization problem
assert traj_is_safe(result.GetTraj(), robot) # Check that trajectory is collision free
trajopt_pub = rospy.Publisher('trajopt',Float32MultiArray, queue_size=1)	
rospy.init_node('trajopt', anonymous=True)
#rate = rospy.Rate(1)
rate = rospy.Rate(24)

traj = Float32MultiArray()
traj.layout.dim.append(MultiArrayDimension())
traj.layout.dim.append(MultiArrayDimension())
traj.layout.dim[0].label = "height"
traj.layout.dim[1].label = "width"	
traj.layout.dim[0].size = 1#20
traj.layout.dim[1].size = 6
traj.layout.dim[0].stride = 6#120
traj.layout.dim[1].stride = 6
traj.layout.data_offset = 0
traj.data =  [0]*6

dstride0 = traj.layout.dim[0].stride
dstride1 = traj.layout.dim[1].stride
offset = traj.layout.data_offset
new_traj = np.asarray(result.GetTraj())
print new_traj.shape
print "shape: {}".format((np.asarray(result.GetTraj())).shape)
print "shape: {}".format(traj.layout.dim)


while not rospy.is_shutdown():
	for num in range (n_steps):
		for j in range (traj.layout.dim[1].size):
			traj.data[j] = result.GetTraj()[num][j] #traj.data.append(result.GetTraj()); #20*num + i   i*6+ '
			#print "offset : {} data :{}" .format(i*6+ j,traj.data[i*6 + j])
			#print traj.data[offset + i + dstride1*j]
			if (rospy.is_shutdown()):
				break
		trajopt_pub.publish(traj)	
		if (rospy.is_shutdown()):
				break
		rospy.loginfo("pubCount: {}".format(pubCount))
		pubCount= pubCount + 1
		rate.sleep()

