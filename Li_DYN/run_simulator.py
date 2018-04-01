import csv
import time
import os
import subprocess
import sys
import matplotlib.pyplot as plt
import math
import numpy as np

csvfile1 = open('/home/raven/homa_wksp/raven_2/teleop_data/new_test_3.csv')
reader = csv.reader(csvfile1)

os.system("cd /home/raven/homa_wksp/raven_2/Li_DYN/")
os.system("pwd")
in_file = open('./dac_mvel_mpos.txt','w')

runlevel = 0
line_no = 0
headers = reader.next()
#print headers
# Find the indices for the variables in the datasheet
runlevel_index = headers.index('field.runlevel'); 
mpos_index = headers.index('field.mpos0');
dmpos_index = headers.index('field.mpos_d0');
mvel_index = headers.index('field.mvel0');
dmvel_index = headers.index('field.mvel_d0');
dac_index = headers.index('field.current_cmd0');

while not(runlevel == 3):
	line = reader.next()
	runlevel = int(line[runlevel_index])
	#print runlevel
	line_no = line_no + 1

# Get the inputs
mpos = [[],[],[]]
mvel = [[],[],[]]
indices = [0,1,2]
i = 0;
for line in reader:
	in_file.write(line[mpos_index]+'  ')
	in_file.write(line[mpos_index+1]+'  ')
	in_file.write(line[mpos_index+2]+'  ')	
	in_file.write(line[mvel_index]+'  ')
	in_file.write(line[mvel_index+1]+'  ')
	in_file.write(line[mvel_index+2]+'  ')	
	in_file.write(line[dac_index]+'  ')
	in_file.write(line[dac_index+1]+'  ')
	in_file.write(line[dac_index+2]+'\n')	
	# We are going to compare estimated ones, so shift one sample ahead
	if (i > 1):
		for j in range(0,3):			
			mpos[j].append(float(line[mpos_index+indices[j]])*math.pi/180)
			mvel[j].append(float(line[mvel_index+indices[j]])*math.pi/180)
	i = i + 1;
in_file.close()

'''os.system("make")
os.system("pwd")
writer = 'xterm -hold -e ./writer'
two_arm_dyn = 'xterm -hold -e ./two_arm_dyn'
#os.system('xterm -hold -e ./writer & xterm -hold -e ./two_arm_dyn')
env = os.environ.copy()
p1 = subprocess.Popen(writer, shell=True, preexec_fn=os.setsid)
time.sleep(0.1);
p2 = subprocess.Popen(two_arm_dyn, shell=True, preexec_fn=os.setsid)
print "ran"
time.sleep(5);
os.system("killall xterm")
os.system("killall writer")
os.system("killall two_arm_dyn")'''

out_file = open('./output.csv','r')
# Get the estimated results
est_mpos = [[],[],[]]
est_mvel = [[],[],[]]
for line in out_file:
	results = line.strip().split(',')
	for j in range(0,3):
		est_mpos[j].append(float(results[j*2]))
		est_mvel[j].append(float(results[j*2+1]))
out_file.close()

f, axarr = plt.subplots(3, 2)
for j in range(0,3):
	axarr[j, 0].plot(mpos[j], 'b')
	axarr[j, 0].plot(est_mpos[j], 'r')
	axarr[j, 1].plot(mvel[j], 'b')
	axarr[j, 1].plot(est_mvel[j], 'r')
plt.show()
