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
    //log_msg("updateDeviceState %d", currParams->runlevel);
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
#ifdef simulator
        // Code added by Samin on 11/07/2018
#ifndef packetgen
          // Get initial joint positions from input, assign them to the desired jpos
          float temprot [18] = {-0.968002378941,-0.174213960767,0.180612385273,-0.219871729612,0.241933107376,-0.945052802563,0.120945267379,-0.954524934292,-0.272496551275,-0.93315243721,0.191262990236,0.304376482964,0.344187945127,0.231027543545,0.910033464432,0.103736370802,0.953962624073,-0.281414330006};
          float cart_pos [6] = {-77452,-24161,13793,-77623,25864,13439};
          float cart_pos_d [6]  = {-77452,-24161,13793,-77623,25864,13439};
          float jpos [16] = {29.9492835999,90.373336792,22.9568557739,0,5.11032962799,4.20706319809,43.4555168152,51.1591072083,29.9215259552,90.3847808838,22.9222049713,0,-4.35502958298,-3.17343592644,48.013507843,42.5473213196};
          float jpos_d [16] = {29.949464798,90.373336792,22.9568939209,0,5.11036348343,4.20723438263,43.4458198547,51.1495132446,29.9214191437,90.3846817017,22.9222240448,0,-4.35497236252,-3.17353034019,47.9966468811,42.5306816101};
          float mpos [16] = {4078.98022461,11511.7197266,51634.2617188,0,17779.2304688,17789.4902344,-17360.9101562,18211.6816406,4075.19995117,11512.6210938,51559.1992188,0,-17749.3496094,-17754.390625,18157.5898438,-17343.2714844};
          float mpos_d [16] = {4079.00512695,11511.7236328,51634.3476562,0,17779.2304688,17789.4921875,-17360.9980469,18211.5957031,4075.18554688,11512.6054688,51559.234375,0,-17749.3496094,-17754.390625,18157.4375,-17343.421875};
          float mvel [16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
          float mvel_d [16] = {0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0};
          float jvel_d [16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
          float grasp[2] = {1.65100002289,1.58000004292};
          if (firstpacket == 0){
            for (int i = 0; i < NUM_MECH; i++){
              device0->mech[i].pos.x = cart_pos[i*3 + 0];
              device0->mech[i].pos.y = cart_pos[i*3 + 1];
              device0->mech[i].pos.z = cart_pos[i*3 + 2];
              device0->mech[i].pos_d.x = cart_pos_d[i*3 + 0];
              device0->mech[i].pos_d.y = cart_pos_d[i*3 + 1];
              device0->mech[i].pos_d.z = cart_pos_d[i*3 + 2];
              device0->mech[i].ori_d.grasp  = grasp[i];
              for (int j = 0; j < 8; j++){
                  device0->mech[i].joint[j].jpos = jpos[i*8 + j]*M_PI/180;
                  device0->mech[i].joint[j].jpos_d = jpos_d[i*8 + j]*M_PI/180;
                  device0->mech[i].joint[j].jvel_d = jvel_d[i*8+j]*M_PI/180;
              	  device0->mech[i].joint[j].mpos = mpos[i*8+j]*M_PI/180;
              	  device0->mech[i].joint[j].mvel = mvel[i*8+j]*M_PI/180;
              	  device0->mech[i].joint[j].mpos_d = mpos_d[i*8+j]*M_PI/180;
              	  device0->mech[i].joint[j].mvel_d = mvel_d[i*8+j]*M_PI/180;
              }
            }
            log_msg("Initialized pos, ori, jpos, and mpos values in update_device_state\n");
            if (rcvdParams->surgeon_mode == 1)
              firstpacket = 1;
         }

          //log_msg("\nX,Y,Z -arm %d:\n%d,%d,%d\n", 0, rcvdParams->xd[0].x,rcvdParams->xd[0].y, rcvdParams->xd[0].z);
#else
        		if (currParams->last_sequence == 1)
        		{
        		   for (int i = 0; i < NUM_MECH; i++)
        					for (int j = 0; j < 8; j++)
        				  {
        				    device0->mech[i].joint[j].jpos_d = rcvdParams->jpos_d[i*8+j];
        				    device0->mech[i].joint[j].jvel_d = rcvdParams->jvel_d[i*8+j];
  			            device0->mech[i].joint[j].mpos = rcvdParams->mpos_d[i*8+j];
        				    device0->mech[i].joint[j].mvel = rcvdParams->mvel_d[i*8+j];
        				    device0->mech[i].joint[j].mpos_d = rcvdParams->mpos_d[i*8+j];
        				    device0->mech[i].joint[j].mvel_d = rcvdParams->mvel_d[i*8+j];
        				  }
        			 log_msg("Initialized jpos, jvel, mpos, and mvel in update_device_state\n");
        		}
#endif
#endif
#ifdef detector
        		if (currPdevice0->mech[i].joint[j].mpos = mpos[i*8+j];
              	  device0->mech[i].joint[j].mvel = mvel[i*8+j];
              	  device0->mech[i].joint[j].mpos_d = mpos_d[i*8+j];
              	  device0->mech[i].joint[j].mvel_d = mvel[i*8+j];arams->last_sequence == 1)
        		{
        			for (int i = 0; i < NUM_MECH; i++)
        				for (int j = 0; j < 8; j++)
        				{
        				  device0->mech[i].joint[j].jpos_d = rcvdParams->jpos_d[i*8+j];
        					device0->mech[i].joint[j].jvel_d = rcvdParams->jvel_d[i*8+j];
        					device0->mech[i].joint[j].mpos = rcvdParams->mpos_d[i*8+j];
        					device0->mech[i].joint[j].mvel = rcvdParams->mvel_d[i*8+j];
        					device0->mech[i].joint[j].mpos_d = rcvdParams->mpos_d[i*8+j];
        					device0->mech[i].joint[j].mvel_d = rcvdParams->mvel_d[i*8+j];
        				}
        			log_msg("Initialized jpos, jvel, mpos, and mvel in update_device_state\n");
        		}
#endif
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
