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


env = openravepy.Environment()

env.StopSimulation()

env.Load('../data/raven_with_workspace.zae')
#env.Load('/home/uva-dsa1/Downloads/raven_2/RavenDebridement/models/ankush_models/raven2.zae')
#env.Load('../data/myRaven.xml')
env.Load('../data/table.xml')
#env.Load('/home/uva-dsa1/Downloads/raven_2/RavenDebridement/models/noYawModels/ravenII_2arm.xml')



trajoptpy.SetInteractive(args.interactive) # pause every iteration, until you press 'p'. Press escape to disable further plotting
robot = env.GetRobots()[0]
print robot
print robot.GetManipulators()[1].GetEndEffectorTransform()#[0].GetArmIndices()
joint_start = []
joint_target = []
joint_start = [29.9499568939,	90.4525146484,	22.9035243988,	11.5578584671,	7.2847337723,	82.4116973877,	56.6583213806]
#joint_target = [27.1548213959,	88.848487854,	22.617143631,	6.5820627213,	0.1315095425,	64.2381439209]
joint_target = [26.903547287,	89.645614624,	22.6126232147,	-10.7246627808,	0.4170058668,	65.9799118042]
#joint_target = [27.1548213959,	88.848487854,	22.617143631,	6.5820627213,	0.1315095425,	64.2381439209,	4.1166629791]

for i in range (len(joint_target)):
	joint_target[i]= math.radians(joint_target[i])
	joint_start[i]= math.radians(joint_start[i])
#print joint_start
joint_target[2]= joint_target[2]/1000
print joint_target

robot.SetDOFValues(joint_start, robot.GetManipulators()[0].GetArmIndices())

#joint_target = [1.5672114866971969, 1.05318772029876708, -0.12326233461499214, -0.18351784765720369, -0.55512279719114299, -0.10556840419769287]
#joint_target = [1.5672114866971969, 1.5318772029876708, -0.12326233461499214, -0.18351784765720369, -0.55512279719114299, -0.10556840419769287, 1.1]
request = {
"basic_info" : {
    "n_steps" : 20,
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
trajopt_pub = rospy.Publisher('trajopt',Float32MultiArray, queue_size=10)	
rospy.init_node('trajopt', anonymous=True)
rate = rospy.Rate(1)

traj = Float32MultiArray()
traj.layout.dim.append(MultiArrayDimension())
traj.layout.dim.append(MultiArrayDimension())
traj.layout.dim[0].label = "height"
traj.layout.dim[1].label = "width"	
traj.layout.dim[0].size = 20
traj.layout.dim[1].size = 6
traj.layout.dim[0].stride = 120
traj.layout.dim[1].stride = 6
traj.layout.data_offset = 0
traj.data =  [0]*120

dstride0 = traj.layout.dim[0].stride
dstride1 = traj.layout.dim[1].stride
offset = traj.layout.data_offset

print "shape: {}".format((np.asarray(result.GetTraj())).shape)
print "shape: {}".format(traj.layout.dim)

for i in range (traj.layout.dim[0].size):			#len(np.asarray(result.GetTraj()))):
	for j in range (traj.layout.dim[1].size):
		traj.data[i*6+ j] = result.GetTraj()[i][j] #traj.data.append(result.GetTraj());
		print "offset : {} data :{}" .format(i*6+ j,traj.data[i*6 + j])
		#print traj.data[offset + i + dstride1*j]
	
while not rospy.is_shutdown():

	trajopt_pub.publish(traj)
	#rospy.loginfo("publish: {}".format(traj))
	rate.sleep()
with open ('traj_data_raven.csv', 'wb') as csvfile:
	spamwriter = csv.writer (csvfile, delimiter = ',', quotechar = '|', quoting = 		csv.QUOTE_MINIMAL)
	for i in range (len(np.asarray(result.GetTraj()))):
						#print (np.asarray(result.GetTraj()[i][j]))
		for j in range(10*len(result.GetTraj())):
			spamwriter.writerow (np.asarray(result.GetTraj()[i] ))			



