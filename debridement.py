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
import trajoptpy.kin_utils as ku
tissue_position = []
pubCount = 0
n_steps = 20
count = 0
env = openravepy.Environment()
env.StopSimulation()
env.Load('/home/uva-dsa1/raven_2/src/raven/trajopt/data/raven_with_workspace.zae')
env.Load('/home/uva-dsa1/raven_2/src/raven/trajopt/data/table.xml')

trajoptpy.SetInteractive(args.interactive) # pause every iteration, until you press 'p'. Press escape to disable further plotting
current_pose = [31.8029937744,	88.3655548096,	22.5955314636,	-7.0923576355,	23.0211296082,	66.2951507568]
drop_point = [31.8029937744,	88.3655548096,	22.5955314636,	-7.0923576355,	23.0211296082,	84.2381439209]
num_tissues = 3
joint_start = []
joint_target = []
robot = env.GetRobots()[0]
#joint_start = [29.9499568939,	90.4525146484,	22.9035243988,	11.5578584671,	7.2847337723,	82.4116973877,	56.6583213806]
#joint_start = [30, 90,	22.9183120728,	245.224884033,	86.7720489502,	101.590637207]
def setParams(joint_start, joint_target):
	global robot
	
	global count
	if (count==6):
		print 'joint_start :{} joint_taget : {}' .format(joint_start, joint_target)
	
	#print robot
	#print robot.GetManipulators()[1].GetEndEffectorTransform()#[0].GetArmIndices()
	#joint_start[2]= joint_start[2]/1000
	joint_target[2]= joint_target[2]/1000
	print "joint_target[2] : {}" .format(joint_target[2])
	for i in range (len(joint_target)):
		joint_target[i]= math.radians(joint_target[i])
		joint_start[i]= math.radians(joint_start[i])
	
	#print joint_start

	#joint_start[2]= joint_start[2]/1000
	#joint_target[2]= joint_target[2]/1000
	
	count=count+1
	print 'count: {}' .format(count)

	robot.SetDOFValues(joint_start, robot.GetManipulators()[1].GetArmIndices())
	run_trajopt(joint_target)


def run_trajopt(joint_target):
	global robot, pubCount
	#print 'entered trajopt'
	global current_pose
	
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

	#print result
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
	#print new_traj.shape
	#print "shape: {}".format((np.asarray(result.GetTraj())).shape)
	#print "shape: {}".format(traj.layout.dim)
	print 'result.getTraj() : {}'.format(result.GetTraj())
	current_pose = result.GetTraj()[n_steps-1]

	for i in range(len(current_pose)):
		current_pose[i] = math.degrees(current_pose[i])
	print 'current_pose: {}' .format(current_pose)		

	for num in range (n_steps):
		for j in range (traj.layout.dim[1].size):
			
			traj.data[j] = result.GetTraj()[num][j] #traj.data.append(result.GetTraj()); #20*num + i   i*6+ '
			if (j==2):
				traj.data[j]=traj.data[j]*1000
				print traj.data[j]
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
		
def main():
	global current_pose, tissue_position

	for i in range(num_tissues):
				
		tissue_position = [[7.1548213959,	78.848487854,	25.617143631,	76.5820627213,	-71.1315095425,	0],
							[37, 90, 28.9183120728, -55, -45.000007629, 0],
							[67.8020744324,	88.3662567139,	23.5955276489,	-157.0666623116,	53.0164585114,	0],
							[27.8020744324,	88.3662567139,	25.5955276489,	-57.0666623116,	33.0164585114,	0]]
		for k in range(len(tissue_position)):
			setParams(current_pose, tissue_position[k])
			rospy.sleep(0.5)
			drop_point = [31.8029937744,	88.3655548096,	22.5955314636,	-7.0923576355,	23.0211296082,	84.2381439209]
			setParams(current_pose, drop_point)
			rospy.sleep(1)

if __name__ == '__main__':
	
	main()
