'''/* Surgeon Packet Generator - Works with Control software for the Raven II robot
  Input arguments simulator
 * Copyright (C) 2015 University of Illinois Board of Trustees, DEPEND Research Group, Creator: Homa Alemzadeh
 *
 * This file is part of Raven 2 Surgical Simulator.
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

import socket
import struct
import csv
import time
from collections import namedtuple
import threading
import sys
import signal
import time
import math
from sys import argv
import os

UDP_IP = "127.0.0.1"
UDP_PORT1 = 32000
UDP_PORT2 = 36001

# Not_Ready: state = 0, Operating: state = 1, Stopped: state = 2  
robot_state = 0;

script, mode, traj = argv
simulator = 0
if mode == "sim" or mode == "dyn_sim" or mode == 'detect':
    simulator = 1
    print "Packet Generator for Simulation"
elif mode == "rob": 
    simulator = 0
    print "Packet Generator for Robot Replay"
else:
    print "Usage: python Real_Packet_Generator_Surgoen <sim|rob>"
    sys.exit(2)
print "Trajectory: "+str(traj)

# Get raven_home directory
env = os.environ.copy()
splits = env['ROS_PACKAGE_PATH'].split(':')
raven_home = splits[0]
teleop_home = raven_home+'/teleop_data'
print '\nRaven Home Found to be: '+ raven_home

fast_surgeon = 0
if fast_surgeon:
   MAX_PACKETS = 5000
   FREQ = 0.001
else: 
   MAX_PACKETS = 3000
   FREQ = 0.01

# When only running simulator with no robot, we need to send jpos values
if (simulator):
   format_ = "<IIIiiiiiiddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddiiiiii"
   u_struct = namedtuple("u_struct", "sequence pactyp version delx0 delx1 dely0 dely1 delz0 delz1 R_l00 R_l01 R_l02 R_l10 R_l11 R_l12 R_l20 R_l21 R_l22 R_r00 R_r01 R_r02 R_r10 R_r11 R_r12 R_r20 R_r21 R_r22 jpos0 jpos1 jpos2 jpos3 jpos4 jpos5 jpos6 jpos7 jpos8 jpos9 jpos10 jpos11 jpos12 jpos13 jpos14 jpos15 jvel0 jvel1 jvel2 jvel3 jvel4 jvel5 jvel6 jvel7 jvel8 jvel9 jvel10 jvel11 jvel12 jvel13 jvel14 jvel15 mpos0 mpos1 mpos2 mpos3 mpos4 mpos5 mpos6 mpos7 mpos8 mpos9 mpos10 mpos11 mpos12 mpos13 mpos14 mpos15 mvel0 mvel1 mvel2 mvel3 mvel4 mvel5 mvel6 mvel7 mvel8 mvel9 mvel10 mvel11 mvel12 mvel13 mvel14 mvel15 buttonstate0 buttonstate1 grasp0 grasp1 surgeon_mode checksum");
else:
   format_ = "<IIIiiiiiiddddddddddddddddddiiiiii"
   u_struct = namedtuple("u_struct", "sequence pactyp version delx0 delx1 dely0 dely1 delz0 delz1 R_l00 R_l01 R_l02 R_l10 R_l11 R_l12 R_l20 R_l21 R_l22 R_r00 R_r01 R_r02 R_r10 R_r11 R_r12 R_r20 R_r21 R_r22 buttonstate0 buttonstate1 grasp0 grasp1 surgeon_mode checksum");

seq = 0;
sock1 = socket.socket(socket.AF_INET, # Internet
                  socket.SOCK_DGRAM) # UDP
sock1.bind((UDP_IP,UDP_PORT1))
sock2 = socket.socket(socket.AF_INET, # Internet
                  socket.SOCK_DGRAM) # UDP
def readSignals():
    global robot_state
    while(1):
        data = sock1.recvfrom(100)
        if (robot_state == 0):
            print "\rRaven is stopped..."       
            print "\rReceieved "+str(data)+ " from RAVEN"
            if (data[0].find('Ready') > -1):
                robot_state = 1

        elif (robot_state == 1):
            print "\rRaven is ready...",
            print "\rReceieved "+str(data)+ " from RAVEN"
            if (data[0].find('Stopped') > -1):
                robot_state = 2

        elif (robot_state == 2): 
            print "\rReceieved "+str(data)+ " from RAVEN"
            if (data[0].find('Ready') > -1):
                robot_state = 1

def sendPackets():
    global seq
    csvfile = open(teleop_home+'/new_test_'+str(traj.split('traj')[1])+'.csv','r');    
    outfile = open('./robot_run.csv','w');
    #reader = csv.reader(csvfile)
    reader = csv.reader(x.replace('\0', '') for x in csvfile)
    writer = csv.writer(outfile,delimiter=',')
    headers = reader.next()
    writer.writerow(headers)
    # Find the indices for the variables in the datasheet
    runlevel_index = headers.index('field.runlevel');
    packet_index = headers.index('field.last_seq'); 
    dori_index = headers.index('field.ori_d0');
    dpos_index = headers.index('field.pos_d0');
    pos_index = headers.index('field.pos0');
    jpos_index = headers.index('field.jpos0');
    jvel_index = headers.index('field.jvel0');
    dmpos_index = headers.index('field.mpos_d0');
    mpos_index = headers.index('field.mpos0');
    mvel_index = headers.index('field.mvel0');
    grasp_index = headers.index('field.grasp_d0');
    enc_index = headers.index('field.encVals0');
    dac_index = headers.index('field.current_cmd0');
    djpos_index = headers.index('field.jpos_d0');

    # Skip the packets until runlevel 3
    runlevel = 0;	
    packet_num = 111;
    line_no = 0;
    line = [];
    while (runlevel < 3) or (packet_num == 111) or (packet_num == 0):
        try:
            line = reader.next()
        except Exception: 
            print line
        line_no = line_no+1
        runlevel = int(line[runlevel_index])
        packet_num = int(line[packet_index])
    print('Started at Line = '+ str(line_no)+ ' and Packet = '+str(packet_num))
    
    #for i in range(0,SKIP):
    #   line = reader.next()
    
    global robot_state
    while (robot_state == 0):
        pass
        #print('\rWaiting for Raven...')  
  
    # Send trajectory packets until the operation is done (e.g. until runlevel = 3 or for 9000 steps) 
    # Skip duplicate packets in the trajectory  
    while (seq < MAX_PACKETS):
        # If robot is ready and not sent all the packets yet
        if (robot_state == 1): #(runlevel == 3):
            line = reader.next()
            line_no = line_no + 1;
            runlevel = int(line[runlevel_index])
            writer.writerow(line);
            # If a new packet is seen, then process and send it
            if not(int(line[packet_index]) == packet_num):			
                packet_num = int(line[packet_index])		
                # Print the mpos and mvel and write to the file for comparison
                print_line = ''
                for i in range(0,8):
                    print_line = print_line + (str(float(line[dmpos_index+i])*(math.pi/180))+',')	
                    print_line = print_line + (str(float(line[mpos_index+i])*(math.pi/180))+',')
                    print_line = print_line + (str(float(line[mvel_index+i])*(math.pi/180))+',')
                    print_line = print_line + (str(float(line[dac_index+i]))+',')
                    print_line = print_line + (str(float(line[djpos_index+i])*(math.pi/180))+',')
                    print_line = print_line + (str(float(line[jpos_index+i])*(math.pi/180))+',')
                print('(mpos,mvel,dac) = ' + print_line+'\n')			
                for i in range(0,3):
                    print_line = print_line + (str(float(line[dpos_index+i]))+',')
                    print_line = print_line + (str(float(line[pos_index+i]))+',')
            
                
                # Construct the packet to send
                seq = seq + 1;
                print "\nSending Packet #"+str(seq)   
                if (simulator):  
                    tuple_to_send = u_struct(sequence = seq, 
                        pactyp = 0, 
                        version = 0, 
                        delx0 = int(float(line[dpos_index])),
                        delx1 = int(float(line[dpos_index+3])),
                        dely0 = int(float(line[dpos_index+1])),
                        dely1 = int(float(line[dpos_index+4])),
                        delz0 = int(float(line[dpos_index+2])),
                        delz1 = int(float(line[dpos_index+5])),
                        R_l00 = float(line[dori_index]),
                        R_l01 = float(line[dori_index+1]),
                        R_l02 = float(line[dori_index+2]),
                        R_l10 = float(line[dori_index+3]),
                        R_l11 = float(line[dori_index+4]), 
                        R_l12 = float(line[dori_index+5]),
                        R_l20 = float(line[dori_index+6]),
                        R_l21 = float(line[dori_index+7]),
                        R_l22 = float(line[dori_index+8]),
                        R_r00 = float(line[dori_index+9]),
                        R_r01 = float(line[dori_index+10]), 
                        R_r02 = float(line[dori_index+11]),
                        R_r10 = float(line[dori_index+12]),
                        R_r11 = float(line[dori_index+13]), 
                        R_r12 = float(line[dori_index+14]),
                        R_r20 = float(line[dori_index+15]),
                        R_r21 = float(line[dori_index+16]),
                        R_r22 = float(line[dori_index+17]),
                        jpos0 = float(line[jpos_index]),
                        jpos1 = float(line[jpos_index+1]),
                        jpos2 = float(line[jpos_index+2]),
                        jpos3 = float(line[jpos_index+3]),
                        jpos4 = float(line[jpos_index+4]),
                        jpos5 = float(line[jpos_index+5]),
                        jpos6 = float(line[jpos_index+6]),
                        jpos7 = float(line[jpos_index+7]),
                        jpos8 = float(line[jpos_index+8]),
                        jpos9 = float(line[jpos_index+9]),
                        jpos10 = float(line[jpos_index+10]),
                        jpos11 = float(line[jpos_index+11]),
                        jpos12 = float(line[jpos_index+12]),
                        jpos13 = float(line[jpos_index+13]),
                        jpos14 = float(line[jpos_index+14]),
                        jpos15 = float(line[jpos_index+15]),
                        jvel0 = float(line[jpos_index]),
                        jvel1 = float(line[jpos_index+1]),
                        jvel2 = float(line[jpos_index+2]),
                        jvel3 = float(line[jpos_index+3]),
                        jvel4 = float(line[jpos_index+4]),
                        jvel5 = float(line[jpos_index+5]),
                        jvel6 = float(line[jpos_index+6]),
                        jvel7 = float(line[jpos_index+7]),
                        jvel8 = float(line[jpos_index+8]),
                        jvel9 = float(line[jpos_index+9]),
                        jvel10 = float(line[jpos_index+10]),
                        jvel11 = float(line[jpos_index+11]),
                        jvel12 = float(line[jpos_index+12]),
                        jvel13 = float(line[jpos_index+13]),
                        jvel14 = float(line[jpos_index+14]),
                        jvel15 = float(line[jpos_index+15]),
                        mpos0 = float(line[mpos_index]),
                        mpos1 = float(line[mpos_index+1]),
                        mpos2 = float(line[mpos_index+2]),
                        mpos3 = float(line[mpos_index+3]),
                        mpos4 = float(line[mpos_index+4]),
                        mpos5 = float(line[mpos_index+5]),
                        mpos6 = float(line[mpos_index+6]),
                        mpos7 = float(line[mpos_index+7]),
                        mpos8 = float(line[mpos_index+8]),
                        mpos9 = float(line[mpos_index+9]),
                        mpos10 = float(line[mpos_index+10]),
                        mpos11 = float(line[mpos_index+11]),
                        mpos12 = float(line[mpos_index+12]),
                        mpos13 = float(line[mpos_index+13]),
                        mpos14 = float(line[mpos_index+14]),
                        mpos15 = float(line[mpos_index+15]),
                        mvel0 = float(line[mvel_index]),
                        mvel1 = float(line[mvel_index+1]),
                        mvel2 = float(line[mvel_index+2]),
                        mvel3 = float(line[mvel_index+3]),
                        mvel4 = float(line[mvel_index+4]),
                        mvel5 = float(line[mvel_index+5]),
                        mvel6 = float(line[mvel_index+6]),
                        mvel7 = float(line[mvel_index+7]),
                        mvel8 = float(line[mvel_index+8]),
                        mvel9 = float(line[mvel_index+9]),
                        mvel10 = float(line[mvel_index+10]),
                        mvel11 = float(line[mvel_index+11]),
                        mvel12 = float(line[mvel_index+12]),
                        mvel13 = float(line[mvel_index+13]),
                        mvel14 = float(line[mvel_index+14]),
                        mvel15 = float(line[mvel_index+15]),
                        buttonstate1 = 0,
                        buttonstate0 = 0, 		        
                        grasp0 = float(line[grasp_index+0])*1000,
                        grasp1 = float(line[grasp_index+1])*1000,
                        #grasp0 = float((float(line[jpos_index+7])+float(line[jpos_index+6]))*1000),
                        #grasp1 = float((float(line[jpos_index+15])+float(line[jpos_index+14]))*1000), 
                        surgeon_mode = 1, 
                        checksum=0);
                else:
                    tuple_to_send = u_struct(sequence = seq, 
                        pactyp = 0, 
                        version = 0, 
                        delx0 = int(float(line[dpos_index])),
                        delx1 = int(float(line[dpos_index+3])),
                        dely0 = int(float(line[dpos_index+1])),
                        dely1 = int(float(line[dpos_index+4])),
                        delz0 = int(float(line[dpos_index+2])),
                        delz1 = int(float(line[dpos_index+5])),
                        R_l00 = float(line[dori_index]),
                        R_l01 = float(line[dori_index+1]),
                        R_l02 = float(line[dori_index+2]),
                        R_l10 = float(line[dori_index+3]),
                        R_l11 = float(line[dori_index+4]),
                        R_l12 = float(line[dori_index+5]),
                        R_l20 = float(line[dori_index+6]),
                        R_l21 = float(line[dori_index+7]),
                        R_l22 = float(line[dori_index+8]),
                        R_r00 = float(line[dori_index+9]),
                        R_r01 = float(line[dori_index+10]),
                        R_r02 = float(line[dori_index+11]),
                        R_r10 = float(line[dori_index+12]),
                        R_r11 = float(line[dori_index+13]),
                        R_r12 = float(line[dori_index+14]),
                        R_r20 = float(line[dori_index+15]),
                        R_r21 = float(line[dori_index+16]),
                        R_r22 = float(line[dori_index+17]),
                        buttonstate0 = 0,
                        buttonstate1 = 0, 
                        grasp0 = float(line[grasp_index+0])*1000,
                        grasp1 = float(line[grasp_index+1])*1000,
                        #grasp0 = float((float(line[jpos_index+7])+float(line[jpos_index+6]))*1000),
                        #grasp1 = float((float(line[jpos_index+15])+float(line[jpos_index+14]))*1000), 
                        surgeon_mode = 1, 
                        checksum=0);                
                MESSAGE = struct.pack(format_,*tuple_to_send._asdict().values());
                if (robot_state == 1):
                    print struct.unpack(format_,MESSAGE);
                    print "\n"
                # Send the command to the robot
                sock2.sendto(MESSAGE, (UDP_IP, UDP_PORT2))
                time.sleep(FREQ)
            # If in robot_s stay with the same packet but with new seq number
        elif (robot_state == 2):
            #print "\rWaiting for the robot to be restarted",
            sys.stdout.flush();
    seq = -1;
    print "All Packets Sent.."
    csvfile.close()
    outfile.close()
def signal_handler(signal, frame):
    sock1.close()
    sock2.close()
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

t1 = threading.Thread(target=readSignals);
t2 = threading.Thread(target=sendPackets);
t1.daemon = True;
t2.daemon = True;
t1.start();
t2.start();

while(seq > -1):
	time.sleep(FREQ)

sock1.close()
sock2.close()
sys.exit(0)

 



