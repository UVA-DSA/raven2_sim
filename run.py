'''/* Runs Raven 2 simulator by calling packet generator, Raven control software, and visualization code
 * Copyright (C) 2015 University of Illinois Board of Trustees, DEPEND Research Group, Creators: Homa Alemzadeh and Daniel Chen
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

import os
import subprocess
import random
import sys
from math import cos, sin, sqrt, acos, asin, pow as pow_f
import socket
import sys
import numpy as np
import struct
import time
import datetime
import signal
from sys import argv
import logging
import csv
import matplotlib.pyplot as plt
import math
import time
import shelve

def rsp_func():
    """ Get response from user to check if raven_home directory is correct"""
    rsp = str(raw_input("Is the Raven Home found correctly (Yes/No)? "))
    if rsp.lower() == 'yes' or rsp.lower() == 'y':
            print 'Found Raven Home Directory.. Starting..\n'
    elif rsp.lower() == 'no' or rsp.lower() == 'n':
            print 'Please change the ROS_PACKAGE_PATH environment variable.\n'
            sys.exit(2)
    else:
            rsp_func()



class Raven():
    """ Implements the Raven class to run different Raven experiments"""
    def __init__(self, raven_home, mode, packet_gen, trajectory):
        """ Init variables """
        self.mode = mode
        self.packet_gen = packet_gen
        self.traj = trajectory
        self.raven_home = raven_home
        self.shelve_file = raven_home + "/run.shelve"
        self.defines_changed = 0
        self.defines_src_file = raven_home + "/include/raven/defines.h"
        self.defines_bkup_file = raven_home + "/include/raven/defines_back.h"
        self.defines_chk_file = raven_home + "/include/raven/defines_last_run"
        self.trj_name = ''
        self.defines_changed = 0
        self.return_code = 0 #0 is normal, 1 is error
        self.rviz_enabled = 1
        self.result_folder = ''
        self.exp_status = '' # expriment status: 'running' or 'done'

      
    def __change_defines_h(self):
        """ Modifies <raven_home>/include/raven/defines.h """
        # Change define macros
        cmd = 'cp ' + self.defines_src_file + ' ' + self.defines_bkup_file
        os.system(cmd)
        #open files
        src_fp = open(self.defines_src_file,'w')
        bkup_fp = open(self.defines_bkup_file,'r')

        for line in bkup_fp:
            if line.startswith('//#define simulator'):
                if (self.mode == 'sim' or self.mode == 'dyn_sim'):
                    line = line.lstrip('//')
            elif line.startswith('//#define dyn_simulator'):
                if self.mode == 'dyn_sim':
                    line = line.lstrip('//')
            elif line.startswith('//#define packetgen'):
                if self.packet_gen == '1':
                    line = line.lstrip('//')
    
            src_fp.write(line)
        src_fp.close()
        bkup_fp.close()
        #save a check file
        cmd = 'cp ' + self.defines_src_file + ' ' + self.defines_chk_file
        os.system(cmd)
        self.defines_changed = 1

    def __restore_defines_h(self):
        """ Restores <raven_home>/include/raven/defines.h """
        #restore file
        cmd = 'chmod 777 ' + self.defines_bkup_file;
        os.system(cmd);
        cmd = 'cp ' + self.defines_bkup_file + ' ' + self.defines_src_file
        # delete backup
        if (os.system(cmd) == 0): 
            cmd = 'rm ' + self.defines_bkup_file;
            os.system(cmd);   
        self.defines_changed = 0

           
    def __quit(self): 
        """ Terminate all process started by _run_experiment() """
        # Restore changes to source code

        if self.defines_changed:
            self.__restore_defines_h()
        
        try:
            r2_control_pid = subprocess.check_output("pgrep r2_control", 
                    shell=True)
            os.killpg(int(r2_control_pid), signal.SIGINT)
            time.sleep(1)
        except:
            pass
        try:
            roslaunch_pid = subprocess.check_output("pgrep roslaunch", 
                    shell=True)
            os.killpg(int(roslaunch_pid), signal.SIGINT)
            time.sleep(1)
        except:
            pass
        try:
            os.killpg(self.raven_proc.pid, signal.SIGINT)
            time.sleep(1)
        except:
            pass
        try:
            os.killpg(self.packet_proc.pid, signal.SIGINT)
            time.sleep(1)
        except:
            pass
        try:
            os.killpg(self.rostopic_proc.pid, signal.SIGINT)
            time.sleep(1)
        except:
            pass
        try:
            os.killpg(self.dynSim_proc.pid, signal.SIGINT)
            time.sleep(1)
        except:
            pass
        os.system("rm /tmp/dac_fifo > /dev/null 2>&1")
        os.system("rm /tmp/mpos_vel_fifo > /dev/null 2>&1")
        os.system("killall two_arm_dyn > /dev/null 2>&1")        
        os.system("killall r2_control > /dev/null 2>&1")
        os.system("killall roslaunch > /dev/null 2>&1")
        os.system("killall rostopic > /dev/null 2>&1")
        os.system("killall roscore > /dev/null 2>&1")
        os.system("killall rosmaster > /dev/null 2>&1")
        if self.rviz_enabled:
            os.system("killall rviz > /dev/null 2>&1")
        os.system("killall xterm > /dev/null 2>&1")

    def _compile_raven(self):
        """ Compile Raven source code """

        self.__change_defines_h()

        # Make the file
        print "Compiling Raven...logged to compile.output."
        cmd = 'cd ' + self.raven_home + ';make -j 1> compile.output 2>&1'
        make_ret = os.system(cmd)

        if self.defines_changed:
            self.__restore_defines_h()

        if (make_ret != 0):
           print "Make Error: Compilation Failed..\n"
           self.__quit()
           sys.exit(0)

    def _run_experiment(self):
        """ Run Raven experiment once. """

        # Experiment status
        self.exp_status = 'running'


        # Open Sockets
        UDP_IP = "127.0.0.1"
        UDP_PORT = 34000
        sock = socket.socket(socket.AF_INET, # Internet
                              socket.SOCK_DGRAM) # UDP
        sock.bind((UDP_IP,UDP_PORT))

        # Setup Variables
  
        ravenTask = "roslaunch raven_2 raven_2.launch > raven.output"
        visTask = 'xterm -e roslaunch raven_visualization raven_visualization.launch'
        pubTask = 'roslaunch raven_visualization raven_state_publisher.launch'
        dynSimTask = 'xterm -e "cd ./Li_DYN && make -j && ./two_arm_dyn"'
        rostopicTask = 'rostopic echo -p ravenstate >'+self.raven_home+'/latest_run.csv'
        packetTask = 'python '+self.raven_home+'/Real_Packet_Generator_Surgeon.py '+ self.mode + ' '+ self.traj + '> packet_gen.output'

        # Call publisher, visualization, packet generator, and Raven II software
        if self.rviz_enabled:
        	vis_proc = subprocess.Popen(visTask, env=env, shell=True, preexec_fn=os.setsid)
        	time.sleep(2) 
        else:
       		pub_proc = subprocess.Popen(pubTask, env=env, shell=True, preexec_fn=os.setsid)                
        	time.sleep(1) 
        if self.packet_gen == "1":
                self.packet_proc = subprocess.Popen(packetTask, shell=True, preexec_fn=os.setsid)
                print "\nRunning the packet generator.."
        elif self.packet_gen == "0":
                print "\nWaiting for the GUI packets.."
        else:
            print usage
            sys.exit(2)
        self.raven_proc = subprocess.Popen(ravenTask, env=env, shell=True, preexec_fn=os.setsid)
        # Call rostopic to log the data from this RAVEN into latest_run.csv        
        self.rostopic_proc = subprocess.Popen(rostopicTask, env=env, shell=True, preexec_fn=os.setsid)
        time.sleep(0.2);

        # Call Dynamic Simulator
        if self.mode == "dyn_sim":
                self.dynSim_proc = subprocess.Popen(dynSimTask, env=env, shell=True, preexec_fn=os.setsid)
                #os.system("cd ./Li_DYN && make -j && ./two_arm_dyn")
                print "Started the dynamic simulator.."

        print("Press Ctrl+C to exit.")

        #Wait for a response from the robot
        data = ''
        while not data:
            print("Waiting for Raven to be done...")
            data = sock.recvfrom(100)
            if data[0].find('Done!') > -1:
                print("Raven is done, shutdown everything...") 
                self.return_code = 0 
            elif data[0].find('Stopped') > -1:
                print("Raven is E-stopped, shutdown everything...")  
                self.return_code = 1
            else:
                data = ''
        self.exp_status = 'done'
        self.__quit()
  
     
    def _need_compile(self, param):
        myshelve = shelve.open(self.shelve_file)
        if 'param' in myshelve:
            if myshelve['param'] == param:
                print "match!!!"
                return False
            else:
                myshelve['param'] = param
                return True
        else:
            myshelve['param'] = param
            return True
        
                      
        # Delete if result_folder is empty
        try:
            os.rmdir(self.result_folder)
        except OSError as ex:
            pass
    
    def signal_handler(self, signal, frame):
        """ Signal handler to catch Ctrl+C to shutdown everything"""
        print "Ctrl+C Pressed!"
        self.__quit()
        sys.exit(0)

    def run(self):
        """ Run Raven experiments """
	self._compile_raven()  #comment out any time you change mode from rob to sim
	self._run_experiment()
	os.system("python parse_plot.py 0 "+self.traj)

# Main code starts here

# Get raven_home directory
env = os.environ.copy()
splits = env['ROS_PACKAGE_PATH'].split(':')
raven_home = splits[0]
golden_home = raven_home+'/golden_run'
print '\nRaven Home Found to be: '+ raven_home
#rsp_func()
usage = "Usage: python run.py <sim|dyn_sim|rob}> <1:packet_gen|0:gui> <traj2|traj3>"

# Parse the arguments
try:
    script, mode, packet_gen, trajectory = argv
except:
    print "Error: missing parameters"
    print usage
    sys.exit(2)

if mode == "sim":
    print "Run Simulation"
elif mode == "dyn_sim":
    print "Run Dynamic Simulation"
elif mode == "rob": 
    print "Run Real Robot"
else:
    print usage
    sys.exit(2)

# Init Raven
raven = Raven(raven_home, mode, packet_gen, trajectory)
signal.signal(signal.SIGINT, raven.signal_handler)

# Run Raven
raven.run()
