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

#include "update_device_state.h"
#include "log.h"

#ifdef save_logs
extern int logging;
int curr_pack_no = 0;
#endif
//code added by samin 11/07/2018
#ifdef simulator
#ifndef packetgen
extern int firstpacket;
#endif
#endif
//code adding ended

extern struct DOF_type DOF_types[];
extern struct traj trajectory[];
extern int NUM_MECH;
extern volatile int isUpdated;
extern unsigned long gTime;

unsigned int newDofTorqueSetting = 0;   // for setting torque from console
unsigned int newDofTorqueMech = 0;      // for setting torque from console
unsigned int newDofTorqueDof = 0;       //
int newDofTorqueTorque = 0;             // float for torque value in mNm
t_controlmode newRobotControlMode = homing_mode;
const static double d2r = M_PI/180; //degrees to radians
const static double r2d = 180/M_PI; //radians to degrees

/**
 * updateDeviceState - Function that update the device state based on parameters passed from
 *       the user interface
 *
 * \param params_current    the current set of parameters
 * \param arams_update      the new set of parameters
 * \param device0           pointer to device informaiton
 *
 */
int updateDeviceState(struct param_pass *currParams, struct param_pass *rcvdParams, struct device *device0)
{
    //log_file("updateDeviceState %d", currParams->runlevel);///Added
    currParams->last_sequence = rcvdParams->last_sequence;

#ifdef save_logs
    if ((currParams->last_sequence == 111) && (curr_pack_no == 0))
    {
        logging = 0;
    }
    else
    {
	if (currParams->last_sequence != curr_pack_no)
	{
		if (curr_pack_no !=0)
		{
			log_file("______________________________________________\n");
		}
		// Dropped
		if (currParams->last_sequence > (curr_pack_no+1))
		{
  		    for (int i=curr_pack_no+1;i<currParams->last_sequence;i++)
                    {
			log_file("Packet: %d\n", i);
			log_file("Error: Packet Dropped.\n");
			log_file ("______________________________________________\n");
	            }
		}
		curr_pack_no = currParams->last_sequence;
		log_file("Packet: %d\n", curr_pack_no);
	}
    }
#endif

    for (int i = 0; i < NUM_MECH; i++)
    {
        currParams->xd[i].x = rcvdParams->xd[i].x;
        currParams->xd[i].y = rcvdParams->xd[i].y;
        currParams->xd[i].z = rcvdParams->xd[i].z;
        currParams->rd[i].yaw   = rcvdParams->rd[i].yaw;
        currParams->rd[i].pitch = rcvdParams->rd[i].pitch * WRIST_SCALE_FACTOR;
        currParams->rd[i].roll  = rcvdParams->rd[i].roll;
        currParams->rd[i].grasp = rcvdParams->rd[i].grasp;
		// commented debug output
    	//log_msg("Device State: Arm %d : User desired end-effector positions: (%d,%d,%d)", i, currParams->xd[i].x, currParams->xd[i].y, currParams->xd[i].z);
    }

    // set desired mech position in pedal_down runlevel
    if (currParams->runlevel == RL_PEDAL_DN)
    {

#ifdef simulator
// Code added by Samin on 11/07/2018
#ifndef packetgen
  // Get initial joint positions from input, assign them to the desired jpos
  float temprot [18] = {-0.975777983665,-0.207313895226,-0.069844275713,0.00283893872984,0.307242035866,-0.951627194881,0.218744635582,-0.928775131702,-0.299211472273, -0.920559287071,0.22624963522,0.318404972553,0.366252332926,0.216660767794,0.904940545559,0.135756596923,0.949667930603,-0.282313525677};
  float cart_pose [6] = {-77507,-23265,14846,-77477,26066,13309};
  float array [16] = {31.5444812775 DEG2RAD, 90.1361312866 DEG2RAD, 22.9244003296 DEG2RAD, 0, 10.3955860138 DEG2RAD, 19.3089828491 DEG2RAD,-2.37392711639 DEG2RAD, 4.69590997696 DEG2RAD, 29.9757232666 DEG2RAD,90.325050354 DEG2RAD,22.9270248413 DEG2RAD,0,-4.25127267838 DEG2RAD,-3.97164392471 DEG2RAD,46.3926887512 DEG2RAD,45.3422470093 DEG2RAD};

  if (firstpacket ==0){
    for (int i = 0; i < NUM_MECH; i++)

      for (int j = 0; j < 8; j++)

          device0->mech[i].joint[j].jpos_d = array[i*8 + j];
    }
  //log_msg("\nX,Y,Z -arm %d:\n%d,%d,%d\n", 0, rcvdParams->xd[0].x,rcvdParams->xd[0].y, rcvdParams->xd[0].z);
#else

		if (currParams->last_sequence == 1)
		{
			log_msg("I am initizaling jpos, jvel, mpos, and mvel\n");
		    for (int i = 0; i < NUM_MECH; i++)
			{
				for (int j = 0; j < 8; j++)
				{
				    device0->mech[i].joint[j].jpos_d = rcvdParams->jpos_d[i*8+j];
					device0->mech[i].joint[j].jvel_d = rcvdParams->jvel_d[i*8+j];
					device0->mech[i].joint[j].mpos = rcvdParams->mpos_d[i*8+j];
					device0->mech[i].joint[j].mvel = rcvdParams->mvel_d[i*8+j];
					device0->mech[i].joint[j].mpos_d = rcvdParams->mpos_d[i*8+j];
					device0->mech[i].joint[j].mvel_d = rcvdParams->mvel_d[i*8+j];
				}
			}
		}
#endif
#endif
#ifdef detector
		if (currParams->last_sequence == 1)
		{
			log_msg("I am initizaling jpos, jvel, mpos, and mvel\n");
		    for (int i = 0; i < NUM_MECH; i++)
			{
				for (int j = 0; j < 8; j++)
				{
				    device0->mech[i].joint[j].jpos_d = rcvdParams->jpos_d[i*8+j];
					device0->mech[i].joint[j].jvel_d = rcvdParams->jvel_d[i*8+j];
					device0->mech[i].joint[j].mpos = rcvdParams->mpos_d[i*8+j];
					device0->mech[i].joint[j].mvel = rcvdParams->mvel_d[i*8+j];
					device0->mech[i].joint[j].mpos_d = rcvdParams->mpos_d[i*8+j];
					device0->mech[i].joint[j].mvel_d = rcvdParams->mvel_d[i*8+j];
				}
			}
		}
#endif
        for (int i = 0; i < NUM_MECH; i++)
        {
            device0->mech[i].pos_d.x = rcvdParams->xd[i].x;
            device0->mech[i].pos_d.y = rcvdParams->xd[i].y;
            device0->mech[i].pos_d.z = rcvdParams->xd[i].z;
            device0->mech[i].ori_d.grasp  = rcvdParams->rd[i].grasp;

            for (int j=0;j<3;j++)
                for (int k=0;k<3;k++)
	           device0->mech[i].ori_d.R[j][k]  = rcvdParams->rd[i].R[j][k];
        }
    }

    // Switch control modes only in pedal up or init.
    if ( (currParams->runlevel == RL_E_STOP)   &&
         (currParams->robotControlMode != (int)newRobotControlMode) )
    {
        currParams->robotControlMode = (int)newRobotControlMode;
        log_msg("Control mode updated: %d",currParams->robotControlMode);
    }

    // Set new torque command from console user input
    if ( newDofTorqueSetting )
    {
        // reset all other joints to zero
        for (unsigned int idx=0; idx<MAX_MECH_PER_DEV*MAX_DOF_PER_MECH; idx++)
        {
            if ( idx == MAX_DOF_PER_MECH*newDofTorqueMech + newDofTorqueDof )
                currParams->torque_vals[idx] = newDofTorqueTorque;
            else
                currParams->torque_vals[idx] = 0;
        }
        newDofTorqueSetting = 0;
        log_msg("DOF Torque updated\n");
    }

    // Set new surgeon mode
    if ( device0->surgeon_mode != rcvdParams->surgeon_mode)
    {
        device0->surgeon_mode=rcvdParams->surgeon_mode; //store the surgeon_mode to DS0
    }
#ifdef save_logs
    //log_file("Surgoen mode = %d\n",device0->surgeon_mode);
    //log_msg("-----> Runlevel = %d\n",currParams->runlevel);
    //log_file("Current Control Mode: %d\n",currParams->robotControlMode);
#endif

    return 0;
}

/**
*  setRobotControlMode()
*       Change controller mode, i.e. position control, velocity control, visual servoing, etc
*   \param t_controlmode    current control mode.
*/
void setRobotControlMode(t_controlmode in_controlMode){
    newRobotControlMode = in_controlMode;
    log_msg("New control mode: %d",newRobotControlMode);
    isUpdated = TRUE;
}

/**
*  setDofTorque()
*    Set a torque to output on a joint.
*     Torque input is mNm
*   \param in_mech      Mechinism number of the joint
*   \param in_dof       DOF number
*   \param in_torque    Torque to set the DOF to (in mNm)
*/
void setDofTorque(unsigned int in_mech, unsigned int in_dof, int in_torque){
    if (    ((int)in_mech < NUM_MECH)        &&
            ((int)in_dof  < MAX_DOF_PER_MECH) )
    {
        newDofTorqueMech    = in_mech;
        newDofTorqueDof     = in_dof;
        newDofTorqueTorque  = in_torque;
        newDofTorqueSetting = 1;
    }
    isUpdated = TRUE;
}
