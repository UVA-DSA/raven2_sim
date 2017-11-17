/* Raven 2 Control - Control software for the Raven II robot
 * Copyright (C) 2005-2012  H. Hawkeye King, Blake Hannaford, and the University of Washington BioRobotics Laboratory
 *
 * This file is part of Raven 2 Control.
 *
 * Raven 2 Control is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Raven 2 Control is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with Raven 2 Control.  If not, see <http://www.gnu.org/licenses/>.
 */

/**
* \file rt_raven.cpp
* \author Hawkeye
* \version 10/2011
* \brief raven control functions
*
* \ingroup control
*
*   Runs all raven control functions.
*   This a thread run in parallel with rt_process_preempt in order to provide more flexibility.
*
*/

#include <ros/ros.h>  //ROS libraries

#include "rt_raven.h"
#include "defines.h"
#include "init.h"             // for initSurgicalArms()
#include "inv_kinematics.h"
#include "r2_kinematics.h"
#include "inv_cable_coupling.h"
#include "state_estimate.h"
#include "pid_control.h"
#include "grav_comp.h"
#include "t_to_DAC_val.h"
#include "fwd_cable_coupling.h"
#include "fwd_kinematics.h"
#include "trajectory.h"
#include "homing.h"
#include "local_io.h"
#include "update_device_state.h"
#include "parallel.h"
//added include for python reader
#include <python2.7/Python.h>
#include <string>
#include <sstream>
#include <vector>
#include <fstream>
#include <iostream>
#include <assert.h>
#include <stdlib.h>



#ifdef packetgen
extern int done_homing;
#endif
extern int NUM_MECH; //Defined in rt_process_																								preempt.cpp
int counter = 0;
int t_start = 0;
double new_Array[10][8]={0.0};
extern unsigned long int gTime; //Defined in rt_process_preempt.cpp
extern struct DOF_type DOF_types[]; //Defined in DOF_type.h
extern t_controlmode newRobotControlMode; //Defined in struct.h

int raven_cartesian_space_command(struct device *device0, struct param_pass *currParams);
int raven_joint_velocity_control(struct device *device0, struct param_pass *currParams);
int raven_motor_position_control(struct device *device0, struct param_pass *currParams);
int raven_homing(struct device *device0, struct param_pass *currParams, int begin_homing=0);
int applyTorque(struct device *device0, struct param_pass *currParams);
int raven_sinusoidal_joint_motion(struct device *device0, struct param_pass *currParams);
int raven_trajopt_joint_motion(struct device *device0, struct param_pass *currParams, double (&myArray)[10][8]);
int joint();


extern int initialized; //Defined in rt_process_preempt.cpp
//****************************************** "adding newcode for python integration"
std::ifstream data ("/home/uva-dsa1/Downloads/trajopt/python_examples/traj_data.csv");
//******************************************

/**
*  \brief Implements control for one loop cycle.
*
* \post encoders values have been read, runlevel has been set
* \pre robot state is reflected in device0, DAC outputs are set in device0
*
* \brief This funtion controls the Raven based on the desired control mode.
* \param device0 robot_device struct defined in DS0.h
* \param currParam param_pass struct defined in DS1.h
*
* This function first initializes the robot and then it computes Mpos and Velocities by calling
* stateEstimate() and then it calls fwdCableCoupling() and r2_fwd_kin() to calculate the forward cable coupling
* inverse kinematics respectively. 
* The following types of control can be selected from the control mode:
*  -No control
*  -Cartesian Space Control
*  -Motor PD Control
*  -Joint Velocity Control
*  -Homing mode
*  -Applying arbitrary torque
*
*/
int controlRaven(struct device *device0, struct param_pass *currParams){ 

    int ret = 0;
    
    
    //Desired control mode
    t_controlmode controlmode = (t_controlmode)currParams->robotControlMode;
    //std::cout <<"step1" <<std::endl;
    //Initialization code

    initRobotData(device0, currParams->runlevel, currParams);
    //std::cout <<"step2" <<std::endl;
    //Compute Mpos & Velocities
    stateEstimate(device0); 
    //std::cout <<"step3" <<std::endl;
	// commented debug output
    /*log_msg("User desired end-effector positions: Arm %d(%d,%d,%d)\n Arm %d(%d,%d,%d)\n",
                device0->mech[0].type,  device0->mech[0].pos_d.x, device0->mech[0].pos_d.y,device0->mech[0].pos_d.z,device0->mech[1].type,  device0->mech[1].pos_d.x, device0->mech[1].pos_d.y,device0->mech[1].pos_d.z); */

    //Foward Cable Coupling
    fwdCableCoupling(device0, currParams->runlevel);
    //std::cout <<"step4" <<std::endl;
    //Forward kinematics
    r2_fwd_kin(device0, currParams->runlevel);
    //std::cout <<"step5" <<std::endl;
// code insertion to hardcode controlmode
//************************************************* 
    //controlmode = trajopt;
    //controlmode = multi_dof_sinusoid;
    controlmode = trajopt;
    //controlmode = cartesian_space_control; 
//****************************************************
    switch (controlmode){

        //CHECK ME: what is the purpose of this mode?
        case no_control:
        {
		    //initialized = false;
            
            struct DOF *_joint = NULL;
            struct mechanism* _mech = NULL;
            int i=0,j=0;

            // Gravity compensation calculation
            getGravityTorque(*device0, *currParams);

            while ( loop_over_joints(device0, _mech, _joint, i,j) )
                _joint->tau_d = _joint->tau_g;  // Add gravity torque
		
            TorqueToDAC(device0);

        	break;
        }
        //Cartesian Space Control is called to control the robot in cartesian space
	//This is where I might have to insert code, however I should not be need packet_gen
        case cartesian_space_control:
#ifdef packet_gen
                //log_msg("RT_PROCESS) Cartesian space control");         
#endif		
        	ret = raven_cartesian_space_command(device0,currParams);
        	break;
        //Motor PD control runs PD control on motor position
        case motor_pd_control:
            initialized = false;
            ret = raven_motor_position_control(device0,currParams);
            break;
        //Runs joint velocity control
        case joint_velocity_control:
            initialized = false;
            ret = raven_joint_velocity_control(device0, currParams);
            break;
        //Runs homing mode
        case homing_mode:
		
	        static int hom = 0;
        	if (hom==0){
        		log_msg("Entered homing mode");
        		hom = 1;
		
        	}
            initialized = false;

            //initialized = robot_ready(device0) ? true:false;
#ifndef simulator
//#if !defined(simulator) || !defined (autosim)
            ret = raven_homing(device0, currParams);

            set_posd_to_pos(device0);
            updateMasterRelativeOrigin(device0);

            if (robot_ready(device0))
            {
		//code change begins for autosim
                //currParams->robotControlMode = cartesian_space_control;
                //newRobotControlMode = cartesian_space_control;
		//code change ends for autosim , must uncomment this
		//**********************************************************************************
		//currParams->robotControlMode = multi_dof_sinusoid;
		//newRobotControlMode = multi_dof_sinusoid;
		  //  std::cout <<"homingmode" <<std::endl;
		//**********************************************************************************
		// no change due to change of this

	        /*log_msg("Homing Done: end-effector positions: (%d,%d,%d)/(%d,%d,%d)\n",
		device0->mech[0].pos.x, device0->mech[0].pos.y, device0->mech[0].pos.z,
	        device0->mech[1].pos.x, device0->mech[1].pos.y, device0->mech[1].pos.z);*/
            }
#else           
		if (currParams->last_sequence != 1)
		{		
			done_homing = 1; 
			currParams->runlevel = RL_PEDAL_DN; 
        	device0->runlevel = 2;
		}		
		if (currParams->last_sequence == 1)
		{			
		//code also changed here from cartesian_space_control to multi_dof_sinusoid           
	 currParams->robotControlMode = cartesian_space_control;
   	        newRobotControlMode = cartesian_space_control;
		//std::cout <<"homingmode" <<std::endl;
		}
#endif
            break;
	//Runs applyTorque() to set torque command (tau_d) to a joint for debugging purposes
        case apply_arbitrary_torque:
            initialized = false;
            ret = applyTorque(device0, currParams);
            break;
	//Apply sinusoidal trajectory to all joints
        case multi_dof_sinusoid:
            initialized = false;
     	    //std::cout <<"homingmode" <<std::endl;
            ret = raven_sinusoidal_joint_motion(device0, currParams);
            break;
            
	case trajopt:
	    //csv_read();
 	if (counter == 0){
	    csv_read();
	    /*std::ifstream theFile ("/home/uva-dsa1/Downloads/trajopt/python_examples/traj_data.csv");
	    std::string         line;
            std::getline(theFile, line);
            
            int row=0;
            int col=0;
           
          while(std::getline(theFile,line))
	    { 
	    std::stringstream   lineStream(line);
            std::string         cell;
            while(std::getline(lineStream, cell, ','))
            {
                    
                double n = ::atof(cell.c_str());
                if (col==3){
                //std::cout<<new_Array[row][col]<<std::endl;  
                	col=col+1;
                }
                new_Array[row][col]=n;
        	//std::cout<<new_Array[row][col]<<std::endl;        
        	
            col=col+1;
            }
            
                    
            
            std::cout<<row<<std::endl;
            row=row+1;;
            col=0;
            

            if (!lineStream && cell.empty())
            {
                // If there was a trailing comma then add an empty element.
                              //  std::cout <<" cell" << '\n';
                //m_data.push_back("");
            }
        }*/
        //ret=raven_trajopt_joint_motion(device0, currParams, new_Array);
        }
            counter = 1;
	    ret=raven_trajopt_joint_motion(device0, currParams, new_Array);

	    
	    break;

        default:
            ROS_ERROR("Error: unknown control mode in controlRaven (rt_raven.cpp)");
            ret = -1;
            break;
    }
	
#ifdef save_logs
	//log_file("_______________________________________________\n");   
#endif
    return ret;

}

/**
*  \brief  This function runs pd_control on motor position.
*  \param device0 robot_device struct defined in DS0.h
*  \param currParams param_pass struct defined in DS1.h
*  \return -1 if Pedal is up and 0 when torque is applied to DAC
*
* This function:
*  1. calls the r2_inv_kin() to calculate the inverse kinematics
*  2. call the invCableCoupling() to calculate the inverse cable coupling
*  3. set all the joints to zero if pedal is not down otherwise it calls mpos_PD_control() to run the PD control law
*  4. calls getGravityTorque() to calulate gravity torques on each joints.
*  5. calls TorqueToDAC() to apply write torque value's on DAC
* 
*/


/*********************Code for csv starts**************************/




/*********************Code for csv ends*************************/
int raven_cartesian_space_command(struct device *device0, struct param_pass *currParams){
    struct DOF *_joint = NULL;
    struct mechanism* _mech = NULL;
    int i=0,j=0;

    if (currParams->runlevel < RL_PEDAL_UP)
    {
	return -1;
    }
    else if (currParams->runlevel < RL_PEDAL_DN)
    {
        set_posd_to_pos(device0);
    	updateMasterRelativeOrigin(device0);
    }

#ifndef simulator
    parport_out(0x01);
#endif

    //Inverse kinematics
    r2_inv_kin(device0, currParams->runlevel);

	//Inverse Cable Coupling
    invCableCoupling(device0, currParams->runlevel);

    // Set all joints to zero torque
    _mech = NULL;  _joint = NULL;
    while (loop_over_joints(device0, _mech, _joint, i,j) )
    {
        if (currParams->runlevel != RL_PEDAL_DN)
        {
            _joint->tau_d=0;
        }
        else
        {
    	    mpos_PD_control(_joint);
        }
    }

    //log_msg("original tau_d = %f\n",_joint->tau_d);
    // Gravity compensation calculation
    getGravityTorque(*device0, *currParams);
    _mech = NULL;  _joint = NULL;
    while ( loop_over_joints(device0, _mech, _joint, i,j) )
    {
        _joint->tau_d += _joint->tau_g;  // Add gravity torque
    }
    //log_msg("changed tau_d to = %f\n",_joint->tau_d);

    TorqueToDAC(device0);
	/*for (int i =0; i < 2; i++)
	{  
		printf("\nDACs -arm %d:\n%d,%d,%d\n", i, device0->mech[i].joint[SHOULDER].current_cmd,
			  device0->mech[i].joint[ELBOW].current_cmd,
			  device0->mech[i].joint[Z_INS].current_cmd);
		printf("\nX,Y,Z -arm %d:\n%d,%d,%d\n", i, device0->mech[i].pos_d.x,device0->mech[i].pos_d.y,
				  device0->mech[i].pos_d.z);
	}*/
    return 0;
}


/**
*  \brief  This function applies a sinusoidal trajectory to all joints
*  \param device0 is robot_device struct defined in DS0.h
*  \param currParams is param_pass struct defined in DS1.h
*  \return 0 
*
* This function: 
*  1. returns 0 if not in pedal down or init.init (do nothing)
*  2. it sets trajectory on all the joints
*  3. calls the invCableCoupling() to calculate inverse cable coupling
*  4. calls the mpos_PD_control() to run the PD control law
*  5. calls TorqueToDAC() to apply write torque value's on DAC
* 
*/
int raven_sinusoidal_joint_motion(struct device *device0, struct param_pass *currParams){
    static int controlStart = 0;
    static unsigned long int delay=0;
    //const float f_period[8] = {6, 7, 10, 9999999, 10, 5, 10, 6};
    const float f_period[8] = {6, 7, 4, 9999999, 10, 5, 10, 6};
    const float f_magnitude[8] = {10 DEG2RAD, 0 DEG2RAD, 0, 9999999, 0 DEG2RAD, 0 DEG2RAD, 0 DEG2RAD, 0 DEG2RAD};
    //const float f_magnitude[8] = {10 DEG2RAD, 10 DEG2RAD, 0.02, 9999999, 30 DEG2RAD, 30 DEG2RAD, 30 DEG2RAD, 30 DEG2RAD};


/*
    // If we're not in pedal down or init.init then do nothing.
    if (! ( currParams->runlevel == RL_INIT && currParams->sublevel == SL_AUTO_INIT ))
    {
        controlStart = 0;
        delay = gTime;
        // Set all joints to zero torque, and mpos_d = mpos
        for (int i=0; i < NUM_MECH; i++)
        {
            for (int j = 0; j < MAX_DOF_PER_MECH; j++)
            {
                struct DOF* _joint =  &(device0->mech[i].joint[j]);
                _joint->mpos_d = _joint->mpos;
                _joint->jpos_d = _joint->jpos;
                _joint->tau_d = 0;
            }
        }
	std::cout <<"not working" <<std::endl;
        return 0;
    }
*/



    // Wait for amplifiers to power up
    if (gTime - delay < 800)
        return 0;
//*********************** calling the function newJoint();
//******************

    // Set trajectory on all the joints
    for (int i=0; i < NUM_MECH; i++)
    {
        for (int j = 0; j < MAX_DOF_PER_MECH; j++)
        {
            struct DOF * _joint =  &(device0->mech[i].joint[j]);

            int sgn = 1;

            if (device0->mech[i].type == GREEN_ARM)
                sgn = -1;

            // initialize trajectory
            if (!controlStart)
                start_trajectory(_joint, (_joint->jpos + sgn*f_magnitude[j]), f_period[j]);

            // Get trajectory update-********* commented out***
			update_sinusoid_position_trajectory(_joint);
			std::cout<<"entered sinusoid"<<std::endl;
        }
    }

    //Inverse Cable Coupling
    invCableCoupling(device0, currParams->runlevel);
    //std::cout<<"entered inv_coupling"<<std::endl;
    // Do PD control on all the joints
    for (int i=0; i < NUM_MECH; i++)
    {
        for (int j = 0; j < MAX_DOF_PER_MECH; j++)
        {
            struct DOF * _joint =  &(device0->mech[i].joint[j]);

            // Do PD control
            mpos_PD_control(_joint);
//            if (is_toolDOF(_joint))
//            	_joint->tau_d = 0;
        }
    }


    TorqueToDAC(device0);

    controlStart = 1;
    return 0;
}



/**
*  \brief For debugging robot,  apply a set torque command (tau_d) to a joint.
*  \param device0 is robot_device struct defined in DS0.h
*  \param currParams is param_pass struct defined in DS1.h
*  \return 0 
*
* This function: only run in runlevel 1.2
*  1. It checks the run level
*  2. loops over all the joints and mechanisim to set the torque value
*  MAX_DOF_PER_MECH is 8 and is defined in DS0.h 
*  NUM_MECH is the number of mechanisim of the robot
*   
*/
int applyTorque(struct device *device0, struct param_pass *currParams)
{
    // Only run in runlevel 1.2
    if ( ! (currParams->runlevel == RL_INIT && currParams->sublevel == SL_AUTO_INIT ))
        return 0;

    for (int i=0;i<NUM_MECH;i++)
    {
        for (int j=0;j<MAX_DOF_PER_MECH;j++)
        {
            if (device0->mech[i].type == GOLD_ARM)
            {
                device0->mech[i].joint[j].tau_d = (1.0/1000.0) * (float)(currParams->torque_vals[j]);  // convert from mNm to Nm
            }
            else
            {
                device0->mech[i].joint[j].tau_d = (1.0/1000.0) * (float)(currParams->torque_vals[MAX_DOF_PER_MECH+j]);
            }
        }
    }
    // gravComp(device0);
    TorqueToDAC(device0);

    return 0;
}


/**\
*  \brief This function runs PD control on motor position
*  \param device0 is robot_device struct defined in DS0.h
*  \param currParams is param_pass struct defined in DS1.h
*  \return 0 
*
*  This function:
*    1. checks to see if it's in pedal down mode, if not it sets all joints to zero torque and return 0
*    2. set trajectory on all joints
*    3. calls invCableCoupling() to calculate inverse cable coupling
*    4. calls mpos_PD_control() to perform PD control
*    5. calls TorqueToDac() to apply torque on DAC
*/
int raven_motor_position_control(struct device *device0, struct param_pass *currParams)
{
    static int controlStart = 0;
    static unsigned long int delay=0;

    struct DOF *_joint = NULL;
    struct mechanism* _mech = NULL;
    int i=0,j=0;

    // If we're not in pedal down or init.init then do nothing.
    if (! ( currParams->runlevel == RL_PEDAL_DN ||
          ( currParams->runlevel == RL_INIT     && currParams->sublevel == SL_AUTO_INIT ))
       )
    {
        controlStart = 0;
        delay = gTime;

        // Set all joints to zero torque, and mpos_d = mpos
        _mech = NULL;  _joint = NULL;
        while (loop_over_joints(device0, _mech, _joint, i,j) )
        {
            _joint->mpos_d = _joint->mpos;
            _joint->tau_d = 0;
        }
        return 0;
    }

    if (gTime - delay < 800)
        return 0;

    // Set trajectory on all the joints
    _mech = NULL;  _joint = NULL;
    while (loop_over_joints(device0, _mech, _joint, i,j) )
    {
        if ( _joint->type == SHOULDER_GOLD || _joint->type == ELBOW_GOLD )
        	_joint->jpos_d = _joint->jpos;

        if (!controlStart)
            _joint->jpos_d = _joint->jpos;
    }

    //Inverse Cable Coupling
    invCableCoupling(device0, currParams->runlevel);

    // Do PD control on all the joints
    _mech = NULL;  _joint = NULL;
    while (loop_over_joints(device0, _mech, _joint, i,j) )
    {
        // Do PD control
        mpos_PD_control(_joint);

        if (_joint->type < Z_INS_GOLD)
            _joint->tau_d=0;
        else if (gTime % 500 == 0 && _joint->type == Z_INS_GOLD)
        	log_msg("zp: %f, \t zp_d: %f, \t mp: %f, \t mp_d:%f", _joint->jpos, _joint->jpos_d, _joint->mpos, _joint->mpos_d);
    }

    TorqueToDAC(device0);

    controlStart = 1;
    return 0;
}

/**
* \brief This function runs pi_control on joint velocity
* \param device0 is robot_device struct defined in DS0.h
* \param currParams is param_pass struct defined in DS1.h
* \return 0 
*
*  This function:
*  1. if pedal is down or RL_INIT and SL_AUTO_INIT, it Loops over all the joints and all the mechanisim to initialize 
*  velocity trajectory by calling start_trajectory() which is in trajectory.cpp
*  2. calls update_linear_sinusoid_velocity_trajectory() to get the desired joint velocities
*  3. calls jvel_PI_control() to run PI control, which is in pid_control.cpp
*  4. calls TorqueToDac() to apply torque on DAC
*  It sets all the joint torques to zero if pedal is not down or not (RL_INIT and SL_AUTO_INIT)
* 
*/
int raven_joint_velocity_control(struct device *device0, struct param_pass *currParams)
{
    static int controlStart;
    static unsigned long int delay=0;

    // Run velocity control
    if ( currParams->runlevel == RL_PEDAL_DN ||
            ( currParams->runlevel == RL_INIT &&
              currParams->sublevel == SL_AUTO_INIT ))
    {
        // delay the start of control for 300ms b/c the amps have to turn on.
        if (gTime - delay < 800)
            return 0;

        for (int i=0; i < NUM_MECH; i++)
        {
            for (int j = 0; j < MAX_DOF_PER_MECH; j++)
            {
                struct DOF * _joint =  &(device0->mech[i].joint[j]); //initiazed and passed for every joint

                if (device0->mech[i].type == GOLD_ARM)
                {
                    // initialize velocity trajectory
                    if (!controlStart)
                        start_trajectory(_joint);

                    // Get the desired joint velocities
                    update_linear_sinusoid_velocity_trajectory(_joint);

                    // Run PI control
                    jvel_PI_control(_joint, !controlStart);

                }
                else
                {
                    _joint->tau_d = 0;
                }
            }
        }

        if (!controlStart)
            controlStart = 1;

        // Convert joint torque to DAC value.
        TorqueToDAC(device0);
    }

    else
    {
        delay=gTime;
        controlStart = 0;
        for (int i=0; i < NUM_MECH; i++)
            for (int j = 0; j < MAX_DOF_PER_MECH; j++)
            {
                device0->mech[i].joint[j].tau_d=0;
            }
        TorqueToDAC(device0);
    }

    return 0;
}
int raven_trajopt_joint_motion(struct device *device0, struct param_pass *currParams, double (&new_Array)[10][8]){
 static int controlStart = 0;
    static unsigned long int delay=0;
    //const float f_period[8] = {6, 7, 10, 9999999, 10, 5, 10, 6};
    const float f_period[8] = {6, 7, 4, 9999999, 10, 5, 10, 6};
    const float f_magnitude[8] = {10 DEG2RAD, 0 DEG2RAD, 0, 9999999, 0 DEG2RAD, 0 DEG2RAD, 0 DEG2RAD, 0 DEG2RAD};
 

    // Set trajectory on all the joints
    for (int i=0; i < NUM_MECH; i++)
    {
        for (int j = 0; j < MAX_DOF_PER_MECH; j++)
        {
            struct DOF * _joint =  &(device0->mech[i].joint[j]);

            int sgn = 1;

            if (device0->mech[i].type == GREEN_ARM)
                sgn = -1;

            // initialize trajectory
            if (!controlStart){
                start_trajectory(_joint, (_joint->jpos + sgn*f_magnitude[j]), f_period[j]);
		}
		
            // Get trajectory update-********* commented out***
		//start_trajopt(_joint,new_Array[t_start][j]);
		//csv_read();
		//start_trajopt(_joint,j);
		trajopt_with_node(_joint,j);
		//std::cout<<t_start<<std::endl;
        }
    }
    
    
   
    


    //Inverse Cable Coupling
    invCableCoupling(device0, currParams->runlevel);
    //std::cout<<"entered inv_coupling"<<std::endl;
    // Do PD control on all the joints
    for (int i=0; i < NUM_MECH; i++)
    {
        for (int j = 0; j < MAX_DOF_PER_MECH; j++)
        {
            struct DOF * _joint =  &(device0->mech[i].joint[j]);

            // Do PD control
            mpos_PD_control(_joint);
//            if (is_toolDOF(_joint))
//            	_joint->tau_d = 0;
        }
    }


    TorqueToDAC(device0);

    controlStart = 1;
    return 0;
}


