'''/* Runs Raven 2 simulator by calling packet generator, Raven control software, and visualization code
 * Copyright (C) 2015 University of Illinois Board of Trustees, DEPEND Research Group, Creators: Homa Alemzadeh and Daniel Chen
 *
 * This file is part of Raven 2 Surgical Simulator.
 * Provides functions for parsing CSV results files and plotting data
 *
 * Raven 2 Surgical Simulator is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Raven 2 Surgical Simulator is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with Raven 2 Control.  If not, see <http://www.gnu.org/licenses/>.
 */'''

import csv
import time
import os
import subprocess
import sys
import matplotlib.pyplot as plt
import math
import numpy as np 
from sys import argv

def parse_latest_run(reader):
	indices = [0,1,2,4,5,6,7]
	runlevel = 0
	packet_no = 111
	line_no = 0
	headers = reader.next()
	#print headers
	# Find the indices for the variables in the datashee
	runlevel_index = headers.index('field.runlevel'); 
	packet_index = headers.index('field.last_seq'); 
	mpos_index = headers.index('field.mpos0');
	dmpos_index = headers.index('field.mpos_d0');
	mvel_index = headers.index('field.mvel0');
	dmvel_index = headers.index('field.mvel_d0');
	dac_index = headers.index('field.current_cmd0');
	jpos_index = headers.index('field.jpos0');
	djpos_index = headers.index('field.jpos_d0');
	dpos_index = headers.index('field.pos_d0');
	pos_index = headers.index('field.pos0');
	try:
		sim_index = headers.index('field.sim_mpos0');
	except:
		sim_index = -1
	try:
		err_index = headers.index('field.err_msg');
	except:
		err_index = -1

	# Skip the datasheet lines until runlevel = 3 and packet number is 1
	while (runlevel < 3) or (packet_no == 111) or (packet_no == 0):
		line = reader.next()
		runlevel = int(line[runlevel_index])
		packet_no = int(line[packet_index])
		#print runlevel
		line_no = line_no + 1
	print '\rStarted at Line = '+ str(line_no)+ ', Packet = '+str(packet_no)+', Run Level = '+str(runlevel)

	# Get the estimated desired and actual trajectories from the last run 
	est_dmpos = [[],[],[],[],[],[],[]] 
	est_mpos = [[],[],[],[],[],[],[]]
	est_mvel = [[],[],[],[],[],[],[]]
	est_dac = [[],[],[],[],[],[],[]]
	est_djpos = [[],[],[],[],[],[],[]]
	est_jpos = [[],[],[],[],[],[],[]]
	est_dpos = [[],[],[]]
	est_pos = [[],[],[]]
	err_msg = []
	packet_nums = []
	time = []
	sim_mpos = [[],[],[]]
	sim_mvel = [[],[],[]]
	sim_jpos = [[],[],[]]
	
	i = 0
	past_line = ''
	for l in reader:
	# We are going to compare estimated ones, so shift one sample ahead
		if (i > 1) and (int(l[runlevel_index]) == 3):  
			if not(packet_no == int(l[packet_index])):	
				packet_nums.append(packet_no)
				time.append(float(line[0])-t0)
				for j in range(0,7):			
					est_dmpos[j].append(float(line[dmpos_index+indices[j]]))#*math.pi/180)
					est_mpos[j].append(float(line[mpos_index+indices[j]]))#*math.pi/180)
					est_mvel[j].append(float(line[mvel_index+indices[j]]))#*math.pi/180)
				for j in range(0,7):
					est_dac[j].append(float(line[dac_index+indices[j]]))
				for j in range(0,7):
					if j == 2:
						est_djpos[j].append(float(line[djpos_index+indices[j]])*(math.pi/180)*1000)
						est_jpos[j].append(float(line[jpos_index+indices[j]])*(math.pi/180)*1000)
					else:
						est_djpos[j].append(float(line[djpos_index+indices[j]]))#*math.pi/180)
						est_jpos[j].append(float(line[jpos_index+indices[j]]))#*math.pi/180)
				for j in range(0,3):
					est_dpos[j].append(float(line[dpos_index+indices[j]])/1000)#*math.pi/180)
					est_pos[j].append(float(line[pos_index+indices[j]])/1000)#*math.pi/180)
				try:			
					for j in range(0,3):
						sim_mpos[j].append(float(line[sim_index+indices[j]]))
						sim_mvel[j].append(float(line[sim_index+3+indices[j]]))
						sim_jpos[j].append(float(line[sim_index+6+indices[j]]))
				except:
					pass
				try:			
					err_msg.append(str(line[err_index]))
				except:
					pass
			line = l
			packet_no = int(l[packet_index])
		else:
			t0 = float(line[0])
		i = i + 1;

	for j in range(0,3):
		if not(all(v == 0 for v in sim_jpos[j])):
			init_diff = float(est_jpos[j][0]) - float(sim_jpos[j][0])
			sim_jpos[j] = [x+init_diff for x in sim_jpos[j]]
	print len(est_mvel[0])
	print len(est_mpos[0])
	return est_mpos, est_mvel, est_dac, est_jpos, est_pos, sim_mpos, sim_mvel, sim_jpos, err_msg, packet_nums, time 	

#Obselete
def parse_input_data(in_file):
	indices = [0,1,2,4,5,6,7]
	# Get the desired and actural trajectories from the input data
	dmpos = [[],[],[]]
	mpos = [[],[],[]]
	mvel = [[],[],[]]
	dac = [[],[],[],[],[]]
	djpos = [[],[],[],[],[],[],[]]
	jpos = [[],[],[],[],[],[],[]]
	dpos = [[],[],[]]
	pos = [[],[],[]]
	for line in in_file:
		results = line.strip().split(',')
		for j in range(0,7):
			dmpos[j].append(float(results[j*6]))		
			mpos[j].append(float(results[j*6+1]))
			mvel[j].append(float(results[j*6+2]))
		for j in range(0,7):
			dac[j].append(float(results[indices[j]*6+3]))
		for j in range(0,7):
			djpos[j].append(float(results[indices[j]*6+4]))
			jpos[j].append(float(results[indices[j]*6+5]))
		for j in range(0,3):
			dpos[j].append(float(results[48+j*2]))
			pos[j].append(float(results[48+j*2+1]))
	
	return mpos,mvel,dac,jpos,pos

def plot_mpos(m, gold_mpos, mpos, sim_mpos, gold_mvel, mvel, sim_mvel, gold_t, t, mpos_detect, mvel_detect):
	indices = [0,1,2,4,5,6,7]	
	# How many joints to plot
	k = 3
	f1, axarr1 = plt.subplots(k, 2, sharex=True)
	plt.tight_layout()
	axarr1[0,0].set_title("Motor Positions (Gold Arm)")
	axarr1[0,1].set_title("Motor Velocities (Gold Arm)")
	for j in range(0,k):
		axarr1[j, 0].plot(gold_mpos[j], 'g')
		axarr1[j, 0].plot(mpos[j], 'r')
		#axarr1[j, 0].plot(list(np.array(mpos[j][1:])-np.array(mpos[j][:-1])), 'r')
		if j < 3 and not(all(v == 0 for v in sim_mpos[j])):	
			axarr1[j, 0].plot(sim_mpos[j], 'b')	
		if j < 3 and mpos_detect: # and not(all(v == 0 for v in mpos_detect[j])):	
			mpos_vline = min(mpos_detect)# min([i for i, e in enumerate(mpos_detect[j]) if e != 0])
			axarr1[j, 0].axvline(x = mpos_vline, color = 'k', ls = 'dashed')
			#axarr1[j, 0].axvline(x = max(mpos_vlines[j]), color = 'k', ls = 'dashed')
		axarr1[j, 1].plot(gold_mvel[j], 'g')
		axarr1[j, 1].plot(mvel[j], 'r')
		#axarr1[j, 1].plot(list(np.array(mvel[j][1:])-np.array(mvel[j][:-1])), 'r')
		if j < 3 and not(all(v == 0 for v in sim_mvel[j])):	
			axarr1[j, 1].plot(sim_mvel[j], 'b')
		if j < 3 and mvel_detect: #and not(all(v == 0 for v in mvel_detect[j])):	
			mvel_vline = min(mvel_detect)#min([i for i, e in enumerate(mvel_detect[j]) if e != 0]) 
			axarr1[j, 1].axvline(x = mvel_vline, color = 'k', ls = 'dashed')
			#axarr1[j, 1].axvline(x = max(mvel_vlines[j]), color = 'k', ls = 'dashed')			
		# Set the row labels
		axarr1[j, 0].set_ylabel('Motor '+str(indices[j]))
		# Set the Y ticks
		axarr1[j, 0].locator_params(axis = 'y', nbins = 3)
		axarr1[j, 0].tick_params(axis = 'both', labelsize=10)
		# Set the Y ticks
		axarr1[j, 1].locator_params(axis = 'y', nbins = 3)
		axarr1[j, 1].tick_params(axis = 'both', labelsize=10)		
	# Set the column labels
	axarr1[j, 0].set_xlabel('Packet No. (ms)')
	axarr1[j, 1].set_xlabel('Packet No. (ms)')
	plt.tight_layout()	
	#plt.show()
	return f1
  
def plot_dacs(gold_dac, dac, gold_t, t):
	indices = [0,1,2,4,5,6,7]
	k = 3
	f2, axarr2 = plt.subplots(k, 1, sharex=True)
	axarr2[0].set_title("DAC Values (Gold Arm)")
	for j in range(0,k):
		axarr2[j].plot(gold_dac[j], 'g')
		axarr2[j].plot(dac[j], 'r')
		axarr2[j].set_ylabel('Joint '+str(indices[j]))
		# Set the Y ticks
		axarr2[j].locator_params(axis = 'y', nbins = 3)
		axarr2[j].tick_params(axis = 'both', labelsize=10)
	axarr2[j].set_xlabel('Packet No. (ms)')
	plt.tight_layout()
	return f2

def plot_jpos(gold_jpos, jpos, sim_jpos, gold_t, t, jpos_detect):
	indices = [0,1,2,4,5,6,7]
	k = 3
	f3, axarr3 = plt.subplots(k, 1, sharex=True)
	plt.tight_layout()
	axarr3[0].set_title("Joint Positions (Gold Arm)")
	for j in range(0,k):
		axarr3[j].plot(gold_jpos[j], 'g')
		axarr3[j].plot(jpos[j], 'r')
		#axarr3[j].plot(list(np.array(jpos[j][1:])-np.array(jpos[j][:-1])), 'r')
		if j < 3 and not(all(v == 0 for v in sim_jpos[j])):	
			axarr3[j].plot(sim_jpos[j], 'b')			
		if j < 3 and jpos_detect: #and not(all(v == 0 for v in jpos_detect[j])):	
			jpos_vline = min(jpos_detect)#min([i for i, e in enumerate(jpos_detect[j]) if e != 0]) 
			axarr3[j].axvline(x = jpos_vline, color = 'k', ls = 'dashed')
			#axarr3[j].axvline(x = max(jpos_vlines[j]), color = 'k', ls = 'dashed')		
		axarr3[j].set_ylabel('Joint '+str(indices[j]))
		# Set the Y ticks
		axarr3[j].locator_params(axis = 'y', nbins = 3)
		axarr3[j].tick_params(axis = 'both', labelsize=10)
	axarr3[j].set_xlabel('Packet No. (ms)')		
	plt.tight_layout()	
	#plt.show()
	return f3

def plot_pos(gold_pos, pos, gold_t, t,pos_detect):
	indices = [0,1,2,4,5,6,7]
	f4, axarr4 = plt.subplots(3, 1, sharex=True)
	axarr4[0].set_title("End-Effector Positions (Gold Arm)")
	pos_labels = ['X Pos(mm)','Y Pos(mm)','Z Pos(mm)']
	for j in range(0,3):
		axarr4[j].plot(gold_pos[j], 'g')
		axarr4[j].plot(pos[j], 'r')
		#if not(all(v == 0 for v in pos_detect[j])):	
		if pos_detect:
			pos_vline = min(pos_detect)
			#pos_vline = min([i for i, e in enumerate(pos_detect[j]) if e != 0]) 
			axarr4[j].axvline(x = pos_vline, color = 'k', ls = 'dashed')
		axarr4[j].set_ylabel(pos_labels[j])
		axarr4[j].tick_params(axis = 'both', labelsize=10)
	axarr4[j].set_xlabel('Packet No. (ms)')
	plt.tight_layout()	
	#plt.show()
	return f4

def plot_dist(pos, pos_ecludian, pos_detect):
	indices = [0,1,2,4,5,6,7]
	f4, axarr4 = plt.subplots(4, 1, sharex=True)
	axarr4[0].set_title("End-Effector Positions (Gold Arm)")
	pos_labels = ['X Pos(mm)','Y Pos(mm)','Z Pos(mm)']
	for j in range(0,3):
		axarr4[j].plot(pos[j], 'r')
		axarr4[j].set_ylabel(pos_labels[j])
		axarr4[j].tick_params(axis = 'both', labelsize=10)
	axarr4[3].plot(pos_ecludian, 'r')
	axarr4[3].set_ylabel('Ecludian Dist')
	if pos_detect:
		pos_vline = min(pos_detect)
		axarr4[3].axvline(x = pos_vline, color = 'k', ls = 'dashed')
	axarr4[3].set_xlabel('Packet No. (ms)')
	
	'''f4 = plt.figure()
	ax = f4.add_subplot(111)
	ax.plot(pos_ecludian[990:1010], 'r')
	ax.locator_params(axis = 'x', nbins = len(pos_ecludian[990:1010]))'''
	#plt.show()
	return f4

# Main Code Starts Here
print "\nPlotting the results (in ./figures).."
# Get raven_home directory
env = os.environ.copy()
splits = env['ROS_PACKAGE_PATH'].split(':')
raven_home = splits[0]

# Parse the arguments
try:
    script, pmode, traj  = argv
except:
    print "Error: missing parameters"
    print 'python parse_plot.py 0|1 traj_name'
    sys.exit(2)
print 'Mode = '+str(pmode)

# Open Log files
csvfile1 = open(raven_home+'/robot_run.csv')
reader1 = csv.reader(x.replace('\0', '') for x in csvfile1)
# Parse the golden simulator run
gold_mpos, gold_mvel, gold_dac, gold_jpos, gold_pos, gold_sim_mpos, gold_sim_mvel, gold_sim_jpos,gold_err, gold_packets, gold_t = parse_latest_run(reader1)
# Close files
csvfile1.close()

# Parse the latest run of simulator
csvfile2 = open(raven_home+'/latest_run.csv')
reader2 = csv.reader(x.replace('\0', '') for x in csvfile2)
mpos, mvel, dac, jpos, pos, sim_mpos, sim_mvel, sim_jpos, err, packets, t = parse_latest_run(reader2)
# Close files
csvfile2.close()

# Log the results
indices = [0,1,2,4,5,6,7]
posi = ['X','Y','Z']

# Detector: mvel, mpos, jpos
true_detect = [[],[],[],[]]
false_detect = [[],[],[],[]]

# Plot the graphs
cmd = 'mkdir -p ' + raven_home+'/figures'
os.system(cmd)
plot_dacs(gold_dac, dac, gold_t, t).savefig(raven_home+'/figures/dac.png')
plot_mpos(pmode,gold_mpos, mpos, sim_mpos, gold_mvel, mvel, sim_mvel, gold_t, t,true_detect[1], true_detect[0]).savefig(raven_home+'/figures/mpos_mvel.png')
plot_jpos(gold_jpos, jpos, sim_jpos, gold_t, t,true_detect[2]).savefig(raven_home+'/figures/jpos.png')
plot_pos(gold_pos, pos, gold_t, t,true_detect[3]).savefig(raven_home+'/figures/pos.png')

if str(pmode) == '0':
	# Difference between robot and model
	# Write the rows
	if not(os.path.isfile('./sim_robot_results.csv')):
		csvfile7 = open('./sim_robot_results.csv','w')
		writer7 = csv.writer(csvfile7,delimiter=',') 
		#writer7.writerow(['mpos_err0','mvel_err0','jpos_err0','mpos_err1','mvel_err1','jpos_err1','mpos_err2','mvel_err2','jpos_err2'])  
		writer7.writerow(['mpos_err0','jpos_err0','mpos_err1','jpos_err1','mpos_err2','jpos_err2'])  
		csvfile7.close()
	mpos_rob_err = [[],[],[]]
	mvel_rob_err = [[],[],[]]
	jpos_rob_err = [[],[],[]]
	outline = []
	for j in range(0,3):
		if not(all(v == 0 for v in sim_mpos[j])):	
			traj_len = min(len(mpos[j]),len(sim_mpos[j]))
			mpos_diff = list(np.array(mpos[j][0:traj_len])-np.array(sim_mpos[j][0:traj_len]))
			#print 'mpos:'
			#print mpos_diff[1:20]
			for i in range(0, traj_len):
				mpos_diff[i] = abs(mpos_diff[i]/np.array(mpos[j][i]))
			#print mpos_diff[1:20]
			mpos_rob_err[j].append(sum(mpos_diff)/traj_len)
			#print mpos_rob_err[j]
		if not(all(v == 0 for v in sim_mvel[j])):
			traj_len = min(len(mvel[j]),len(sim_mvel[j]))

			#mvel_diff = list(np.array(mvel[j][0:traj_len])-np.array(sim_mvel[j][0:traj_len]))
			#print 'mvel:'
			#print mvel_diff[1:20]
			#for i in range(0, traj_len):
			#	mvel_diff[i] = abs(mvel_diff[i]/np.array(mvel[j][i]))
			#print mvel_diff[1:20]
			#mvel_rob_err[j].append(sum(mvel_diff)/traj_len)	
			#print mvel_rob_err[j]
		
		if not(all(v == 0 for v in sim_jpos[j])):
			traj_len = min(len(jpos[j]),len(sim_jpos[j]))
			jpos_diff = list(np.array(jpos[j][0:traj_len])-np.array(sim_jpos[j][0:traj_len]))
		#	print 'jpos:'
		#	print jpos_diff[1:20]
			for i in range(0, traj_len):
				jpos_diff[i] = abs(jpos_diff[i]/np.array(jpos[j][i]))
			#print jpos_diff[1:20]
			jpos_rob_err[j].append(sum(jpos_diff)/traj_len)	
			#print jpos_rob_err[j]
	
	for j in range(0,3):
		for i in range(0,len(mpos_rob_err[j])):
			#outline.extend([mpos_rob_err[j][i],mvel_rob_err[j][i],jpos_rob_err[j][i]])    
			outline.extend([mpos_rob_err[j][i],jpos_rob_err[j][i]])    
	print outline
	if len(outline) > 0:
		csvfile7 = open('./sim_robot_results.csv','a')
		writer7 = csv.writer(csvfile7,delimiter=',') 
		writer7.writerow(outline)
		csvfile7.close()
