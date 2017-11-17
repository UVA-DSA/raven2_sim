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
import mfi
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

def initLogger(logger, log_file):
    """ Initialize a logger for console and file"""

    fh = logging.FileHandler(log_file)
    fh.setLevel(logging.INFO)

    ch = logging.StreamHandler()
    ch.setLevel(logging.INFO)

    FORMAT = '%(asctime)s - %(message)s'
    formatter = logging.Formatter(FORMAT)
    ch.setFormatter(formatter)
    fh.setFormatter(formatter)

    logger.addHandler(ch)
    logger.addHandler(fh)
    logger.setLevel(logging.INFO)

class Raven():
    """ Implements the Raven class to run different Raven experiments"""
    def __init__(self, raven_home, mode, packet_gen, injection, trajectory):
        """ Init variables """
        self.mode = mode
        self.packet_gen = packet_gen
        self.traj = trajectory
        self.raven_home = raven_home
        self.shelve_file = raven_home + "/run.shelve"
        self.surgeon_simulator = 0
        self.defines_changed = 0
        self.mfi_changed = 0
        self.defines_src_file = raven_home + "/include/raven/defines.h"
        self.defines_bkup_file = raven_home + "/include/raven/defines_back.h"
        self.defines_chk_file = raven_home + "/include/raven/defines_last_run"
        self.master_file = './selected_injection.txt'
        self.inj_line = ''
        self.trj_name = ''
        self.defines_changed = 0
        self.mfi_changed = 0
        self.return_code = 0 #0 is normal, 1 is error
        self.curr_inj = -1
        self.rviz_enabled = 1
        self.result_folder = ''
        self.exp_status = '' # expriment status: 'running' or 'done'
        self.raven_test = raven_home + "/test_raven.py"

        inj = injection.split(':')
        self.injection = inj[0]
        self.starting_inj_num = 0
        self.end_inj_num = -1 
        self.title = ''
        if len(inj) > 1:
            param = inj[1].split('-')
            self.starting_inj_num = int(param[0])
            if len(param) > 1:
                self.end_inj_num = int(param[1])
        
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
                if (self.mode == 'sim' or self.mode == 'dyn_sim') and not(self.mode == 'detect'):
                    line = line.lstrip('//')
            elif line.startswith('//#define dyn_simulator'):
                if self.mode == 'dyn_sim' or self.mode == 'detect':
                    line = line.lstrip('//')
            elif line.startswith('//#define packetgen'):
                if self.packet_gen == '1':
                    line = line.lstrip('//')
            elif line.startswith('//#define mfi'):
                if self.injection == 'mfi' or self.injection == 'mfi2': 
                    line = line.lstrip('//')
            elif line.startswith('//#define detector'):
                if self.mode == 'detect': 
                    line = line.lstrip('//')    
	    elif line.startswith ('//#define autosim'):
		if self.mode =='atsm':
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

    def __mfi_insert_code(self, file_name, mfi_hook, code):
        """ Insert code to <file_name> at location <mfi_hook>"""
        self.mfi_src_file = self.raven_home + "/src/raven/" + file_name
        self.mfi_bkup_file = self.raven_home + "/src/raven/" + file_name + '.bkup'
        self.mfi_chk_file = self.raven_home + "/src/raven/" + file_name + '.chk'

        #save a backup file
        cmd = 'cp ' + self.mfi_src_file + ' ' + self.mfi_bkup_file
        os.system(cmd)
        self.mfi_changed = 1

        #open files
        src_fp = open(self.mfi_src_file, 'w')
        bkup_fp = open(self.mfi_bkup_file, 'r')
        
        for line in bkup_fp:
            src_fp.write(line)
            if line.startswith(mfi_hook):
                src_fp.write(code)
        src_fp.close()
        bkup_fp.close()

        #save a check file
        cmd = 'cp ' + self.mfi_src_file + ' ' + self.mfi_chk_file
        os.system(cmd)

    def __mfi_insert_code2(self, file_name, mfi_hook, trigger, target):
        """ Insert code to <file_name> at location <mfi_hook>
        Example: if (x > 3 && x < 5) {x = 40}
        """
        trigger_line = ' && '.join(trigger)
        # target[0] variable name, target[1] value

        # For R matrices injected values are based on absolute values of yaw, roll, pitch
        if ((target[0] == 'u.R_l') or (target[0] == 'u.R_r')):
            code = 'if (' + trigger_line + ') { '; 
            elems = target[1].split(';');
            for i in range(0,3):
                for j in range(0,3):
                    code =code+target[0]+'['+str(i)+']['+str(j)+']='+ elems[i*3+j]+'; '; 
            code = code + '}\n';  
        # For thetas and USBs the injected value is absolute 
        elif (target[0].find('jpos') > -1) or (file_name.find('USB') > -1):
            code = 'if (' + trigger_line + ') { ' + target[0] + ' = ' + target[1] + ';}\n'
        # For position the injected value is incremental
        else:
            code = 'if (' + trigger_line + ') { ' + target[0] + '+= ' + target[1] + ';}\n'

        self.__mfi_insert_code(file_name, mfi_hook, code)
        return (file_name + ':' + mfi_hook + ':' + code)

    def __restore_mfi(self):
        """ Restores the source file which changed by __mfi_insert_code()"""
        #restore file
        cmd = 'chmod 777 '+self.mfi_bkup_file;
        os.system(cmd);
        cmd = 'cp ' + self.mfi_bkup_file + ' ' + self.mfi_src_file

        # delete backup
        if (os.system(cmd) == 0): 
            cmd = 'rm ' + self.mfi_bkup_file;
            os.system(cmd);   
        self.mfi_changed = 0

    def __copy_files(self):
        # Save latest_run.csv to result_folder for faultfree run
        if self.injection == 'mfi2' and self.exp_status == 'done':
            cmd = 'cp latest_run.csv ' + self.result_folder + '/' + \
                    str(self.curr_inj).zfill(4) + '.csv'
            os.system(cmd)
            cmd = 'cp mfi2.txt ' + self.result_folder
            os.system(cmd)
            cmd = 'cp mfi2_params.csv ' + self.result_folder + '/' + self.param_name     
            os.system(cmd)     
            cmd = 'cp ./robot_run.csv ' + self.result_folder + '/' + self.traj_name
            #cmd = 'cp ./golden_run/traj2.csv ' + self.result_folder + '/' + self.traj_name
            os.system(cmd)
            
    def __quit(self): 
        """ Terminate all process started by _run_experiment() """
        # Restore changes to source code

        if self.defines_changed:
            self.__restore_defines_h()
        if self.mfi_changed:
            self.__restore_mfi()
        
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
        os.system("killall roslaunch > /dev/null 2>&1")
        os.system("killall rostopic > /dev/null 2>&1")
        os.system("killall r2_control > /dev/null 2>&1")
        os.system("killall roscore > /dev/null 2>&1")
        os.system("killall rosmaster > /dev/null 2>&1")
        if self.rviz_enabled:
            os.system("killall rviz > /dev/null 2>&1")
        os.system("killall xterm > /dev/null 2>&1")
        os.system("killall two_arm_dyn > /dev/null 2>&1")
        #os.system("killall python") # Don't work with run_mfi_experiment()

    def _compile_raven(self):
        """ Compile Raven source code """

        self.__change_defines_h()

        # Make the file
        print "Compiling Raven...logged to compile.output."
        cmd = 'cd ' + self.raven_home + ';make -j 1> compile.output 2>&1'
        make_ret = os.system(cmd)

        if self.defines_changed:
            self.__restore_defines_h()
        if self.mfi_changed:
            self.__restore_mfi()

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
        
        ravenTask = "xterm -hold -e 'roslaunch raven_2 raven_2.launch'"
        #ravenTask = "xterm -hold -e 'LD_PRELOAD=/home/raven/homa_wksp/malicious_wrapper/malicious_wrapper.so roslaunch raven_2 raven_2.launch'"
        visTask = 'xterm -e roslaunch raven_visualization raven_visualization.launch'
        pubTask = 'roslaunch raven_visualization raven_state_publisher.launch'
        dynSimTask = 'xterm -e "cd ./Li_DYN && make -j && ./two_arm_dyn"'
        rostopicTask = 'rostopic echo -p ravenstate >'+self.raven_home+'/latest_run.csv'
        trajtask = 'python ' + self.raven_test
        proc=subprocess.call('pwd')
        time.sleep(1)
        #new_terminals=os.listdir('/dev/pts')
        #term=os.path.join('/dev/pts',1)
        

        if (self.surgeon_simulator == 1):
            packetTask = 'xterm -e python '+self.raven_home+'/Real_Packet_Generator_Surgeon.py '+ self.mode + ' '+ self.traj 
            #print(packetTask)
        else:
            packetTask = 'xterm -e python '+self.raven_home+'/Packet_Generator.py'

        # Call publisher, visualization, packet generator, and Raven II software
        if self.rviz_enabled:
        	vis_proc = subprocess.Popen(visTask, env=env, shell=True, preexec_fn=os.setsid)
        	time.sleep(2) 
        else:
       		pub_proc = subprocess.Popen(pubTask, env=env, shell=True, preexec_fn=os.setsid)                
        	time.sleep(1) 
        if self.packet_gen == "1":
                self.packet_proc = subprocess.Popen(packetTask, shell=True, preexec_fn=os.setsid)
                print "Using the packet generator.."
        elif self.packet_gen == "0":
                print "Waiting for the GUI packets.."
        else:
            print usage
            sys.exit(2)
        self.raven_proc = subprocess.Popen(ravenTask, env=env, shell=True, preexec_fn=os.setsid)
        # Call rostopic to log the data from this RAVEN into latest_run.csv        
        self.rostopic_proc = subprocess.Popen(rostopicTask, env=env, shell=True, preexec_fn=os.setsid)
        time.sleep(0.2);

        # Call Dynamic Simulator
        if self.mode == "dyn_sim" or self.mode == "detect":
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
  
        
    def _run_mfi_experiment(self):
        """ Run mfi experiment according to the master_file """
        cur_inj = -1
        saved_param = []

        with open(self.master_file) as fp:
            target_file = ''
            mfi_hook = ''
            trigger = []
            target = []

            for line in fp:
                # Strip '\n' from each line then split by ','
                line = line.strip('\n')
                param = line.split(',')

                # Skip lines begin with # or empty line
                if param[0] == '' or param[0] == '#':
                    continue
               
                # Read location info
                elif param[0] == 'location':
                    location_info = param[1].split(':')
                    target_file = location_info[0].lstrip()
                    mfi_hook = location_info[1]

                # Read trigger info
                elif param[0] == 'trigger':
                    param.pop(0)
                    trigger = [item.strip() for item in param]

                elif param[0] == 'target_r':
                    param.pop(0)
                    saved_param = param
                    target = (mfi.generate_target_r(saved_param)).split(' ')

                elif param[0] == 'injection':
                    if cur_inj != int(param[1]):
                        cur_inj = int(param[1])
                        print("mfi: setup injection %d" % cur_inj)
                    else:
                        # Injection starts at argv[1]
                        # Example starting_inj_num is 3.2
                        if int(param[1]) >= self.starting_inj_num:
                            # If param == 3, indicate do random injection param[2] times.
                            if len(param) == 3:
                                for x in xrange(int(param[2])):
                                    #target = (mfi.generate_target_r(saved_param)).split(' ')
                                    target = (mfi.generate_target_r_stratified(saved_param, int(param[2]), x)).split(' ')
                                    inj_info = self.__mfi_insert_code2(target_file, mfi_hook, trigger, target)
                                    logger.info("injecting to %d.%d %s" % (cur_inj, x, inj_info))
                                    self._compile_raven()
                                    self._run_experiment()
                                    self._run_plot()
                            else:
                                inj_info = self.__mfi_insert_code2(target_file, line, trigger, target)
                                logger.info("injecting to %d %s" % (cur_inj, inj_info))
                                self._compile_raven()
                                self._run_experiment()
                                self._run_plot()

    def __setup_result_folder(self):
        """Create a folder to store the mfi2_experiment results."""
        time = datetime.datetime.now().strftime("%Y%m%d_%H%M")
        if self.title == 'mfi2_empty_test':
            self.result_folder = self.raven_home + '/exp_result/fault_free/' + self.title.rstrip() + '_' + time+'_'+self.mode
        else:
            self.result_folder = self.raven_home + '/exp_result/faulty/' + self.title.rstrip() + '_' + time+'_'+self.mode            
        self.traj_name = time + '.trj'
        self.param_name = time+ '.param'
        cmd = 'mkdir -p ' + self.result_folder
        os.system(cmd)

    def __run_parse_plot(self, mode, inj_num,traj):
        """Call parse_plot.py and copy figures folder to exp_result folder."""
        print "copy figures" #dchen8
        duration = 0
        value = 0.0
        with open('mfi2_params.csv', 'r') as infile:
            reader = csv.reader(infile)
            for line in reader:
                if int(line[0]) == inj_num:
                    duration = line[3]
                    value = line[4]
                    break
        os.system("python parse_plot.py " + str(mode) + " " + str(inj_num) + " "+str(traj))
        cmd = 'cp -r figures/ ' + self.result_folder + '/inj' + '_'.join([str(inj_num), duration, value])
        os.system(cmd)

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
        

    def _run_mfi2_experiment(self):
        try:
            os.remove(self.shelve_file)
        except:
            pass
        """ New mfi injection using the file generated by generate_mfi2.py"""
        code_file = 'mfi2.txt'
        file_name = ''
        mfi_hook = ''
        with open(code_file, 'r') as infile:
            """ Example lines:
                location:network_layer.cpp://MFI_HOOK
                injection 1:if(u.sequence>1000 && u.sequence<1100) {u.del[0]=100;}
            """
            for line in infile:
                self.inj_line = line;
                l = line.split(':')
                if l[0].startswith('injection'):
                    curr_inj = int(l[0].split(' ')[1])
                    self.curr_inj = curr_inj
                    if curr_inj >= self.starting_inj_num:
                        self.__mfi_insert_code(file_name, mfi_hook, l[1])
                        logger.info(line)
                        if self._need_compile(l[1]):
                            self._compile_raven()
                        else:
                            # Skip compile for faultfree runs
                            print "skip compiling!!!"
                            time.sleep(7)
                        self._run_experiment()
                        if self.title == 'mfi2_empty_test':
                            self.__run_parse_plot(0, self.curr_inj,self.traj)
                        else:
                            self.__run_parse_plot(1, self.curr_inj,self.traj)
                        self.__copy_files()
                        if self.curr_inj == self.end_inj_num:
                            break
                elif l[0].startswith('location'):
                    file_name = l[1]
                    mfi_hook = l[2]
                    logger.info("Location: %s:%s" % (file_name, mfi_hook))
                elif l[0].startswith('title'):
                    self.title = l[1].rstrip()
                    logger.info("Experiment Title: " + self.title)
                    self.__setup_result_folder()
                       
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
        if self.injection == 'mfi':
            self._run_mfi_experiment()
        elif self.injection == 'mfi2':
            self._run_mfi2_experiment()
        else:
            self._compile_raven()  #comment out any time you change mode from rob to sim
            self._run_experiment()
            #self.__run_parse_plot(0,-1)
            os.system("python parse_plot.py 0 -1 "+self.traj)

# Main code starts here
#code added


#code adding ended
# Init Logger
logger = logging.getLogger(__name__)
initLogger(logger, 'mfi.log')

# Get raven_home directory
env = os.environ.copy()
splits = env['ROS_PACKAGE_PATH'].split(':')
raven_home = splits[0]
golden_home = raven_home+'/golden_run'
print '\nRaven Home Found to be: '+ raven_home
#rsp_func()
usage = "Usage: python run.py <sim|dyn_sim|rob|detect|atsm}> <1:packet_gen|0:gui> <none|mfi:start#|mfi2:start#> <traj2|traj3>"

# Parse the arguments
try:
    script, mode, packet_gen, injection, trajectory = argv
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
elif mode == "detect": 
    print "Run Real Robot with Dynamic Model Detector"
elif mode == "atsm":
    print "Run the robot in autonomous"
else:
    print usage
    sys.exit(2)

# Init Raven
raven = Raven(raven_home, mode, packet_gen, injection, trajectory)
signal.signal(signal.SIGINT, raven.signal_handler)

# Run Raven
raven.run()
