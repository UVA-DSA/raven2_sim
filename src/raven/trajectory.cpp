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
*    \file trajectory.h
*    \author  Hawkeye
*    \version 10/2011
*    \ingroup Control
*    Generate joint and cartesian trajectories.
*    Internal data structures track trajectory state, and update DOFs as needed upon calling.
*/

#include <ros/ros.h>

#include "trajectory.h"
#include "log.h"
#include "utils.h"
#include "defines.h"

#include <string>
#include <sstream>
#include <vector>
#include <fstream>
#include <iostream>
#include <assert.h>
#include <stdlib.h>
#include <listener.h>
int reqCount = 0;
int readCount = 0;
int count =0;
double Array[4000][8]={0.0};
double subArray[8]= {0.0};
extern unsigned long int gTime;
float joint_val [2][9];	
int i=0;
int c=0;
bool isSub = false;
bool prompt = false;

// Store trajectory parameters
/**    Holds trajectory paramters
 *
 *   \ingroup Control
 */
struct _trajectory
{
   /*@{*/
    ros::Time startTime;   /**<must be in ROS's ros::Time format             */
    float end_pos;         /**<Final position (units are context dependent)  */
    float magnitude;       /**<Amplitude of a sinusoidal trajectory          */
    float period;          /**<Period of sinusoid (seconds)                  */
    float startPos;        /**<Starting position                             */
    float startVel;        /**<Initial velocity                              */
   /*@{*/

};
struct _trajectory trajectory[MAX_MECH*MAX_DOF_PER_MECH];


/**
*    initialize trajectory parameters. Magnitude is set according to difference between _endPos and current joint position.
*
*   \ingroup Control
*
*   \param  _joint    DOF struct for specific joint
*   \param  _endPos   ending position
*   \param _period    duration ( of one cycle)
*
*  The following types of trajectories can be generated (all using the global variable trajectory).  There are two ways to start all trajectories,
*
*  - start_trajectory(), a trajectory relative to current position and velocity with specified ending position
*  - start_trajectory_mag(),  a trajectory relative to current position and velocity with specified magnitude
*
*
*Then each trajectory is updated at each control cycle  in one of 5 ways;
*   -# Sinusoidal Velocity    SHOULDER_GOLD only (update_sinusoid_velocity_trajectory())
*   -# Sinusoidal Velocity    GOLD arm 1st three joints only (update_linear_sinusoid_velocity_trajectory())
*   -# Sinusoidal Position    All joints (update_sinusoid_position_trajectory())
*   -# Single 1/2 cycle           All joints (update_linear_sinusoid_position_trajectory())
*   -# Single full cycle      All joints (update_position_trajectory())
*
* \todo The trajectory generator seems to have a lot of hacks and special cases.  Need more general refactoring.  Also consider polynomial trajectories.
*
*/
int start_trajectory(struct DOF* _joint, float _endPos, float _period)
{
	joint_val [0][0] = 29.9499568939;
	joint_val [0][1] = 90.4525146484;
	joint_val [0][2] = 22.9035243988;
	joint_val [0][3] = 0;
	joint_val [0][4] = 11.5578584671;
	joint_val [0][5] = 7.2847337723;
	joint_val [0][6] = 82.4116973877;
	joint_val [0][7] = 56.6583213806;
	
							

	joint_val [1][0] = 29.9730834961;
	joint_val [1][1] = 90.3447113037;
	joint_val [1][2] = 22.9268455505;
	joint_val [1][3] = 0;
	joint_val [1][4] = -4.5915288925;
	joint_val [1][5] = -3.7058007717;
	joint_val [1][6] = 45.6687583923;
	joint_val [1][7] = 45.258266449;

	
    //dstart_trajectory();
    float j=0;
    if (count>=8){
  	i=1;
	

	j = joint_val[i][count-8];
    }
    else {
	i=0;
	j = joint_val[i][count];
    }

    
    trajectory[_joint->type].startTime = trajectory[_joint->type].startTime.now();
    //trajectory[_joint->type].startPos = _joint->jpos;
    trajectory[_joint->type].startPos = j DEG2RAD;
    //std::cout<<"trajectory[_joint->type].startPos"<<std::endl;
    //std::cout<<trajectory[_joint->type].startPos<<std::endl;
    trajectory[_joint->type].startVel = _joint->jvel;
    
    //_joint->jpos_d = _joint->jpos;
    _joint->jpos_d = j DEG2RAD;
    _joint->jvel_d = _joint->jvel;
    count++;
    trajectory[_joint->type].magnitude = _endPos - _joint->jpos;
  
  
    trajectory[_joint->type].period = _period;
      //std::cout<<_joint->jpos<<std::endl;
//    log_msg("starting trajectory on joint %d to magnitude: %0.3f (%0.3f - %0.3f), period:%0.3f",
//        _joint->type,
//        trajectory[_joint->type].magnitude,
//        _endPos, _joint->jpos,
//        trajectory[_joint->type].period);
	
    return 0;
}

int dstart_trajectory(){

	joint_val [0][0] = 29.9499568939;
	joint_val [0][1] = 90.4525146484;
	joint_val [0][2] = 22.9035243988;
	joint_val [0][3] = 0;
	joint_val [0][4] = 11.5578584671;
	joint_val [0][5] = 7.2847337723;
	joint_val [0][6] = 82.4116973877;
	joint_val [0][7] = 56.6583213806;


	joint_val [1][0] = 29.9499568939;
	joint_val [1][1] = 90.4525146484;
	joint_val [1][2] = 22.9035243988;
	joint_val [1][3] = 0;
	joint_val [1][4] = 11.5578584671;
	joint_val [1][5] = 7.2847337723;
	joint_val [1][6] = 82.4116973877;
	joint_val [1][7] = 56.6583213806;
	joint_val [1][8] = 56.6583213806;
    
//    log_msg("starting trajectory on joint %d to magnitude: %0.3f (%0.3f - %0.3f), period:%0.3f",
//        _joint->type,
//        trajectory[_joint->type].magnitude,
//        _endPos, _joint->jpos,
//        trajectory[_joint->type].period);
return 0;

}



/**
*  initialize trajectory parameters.
*
*  \ingroup Control
*  Start of this trajectory will be the current state: i.e. the position and velocity at this time.
*
*   \param  _joint    DOF struct for specific joint
*   \param  _mag      how big a move
*   \param _period    duration ( of one cycle)
*/
int start_trajectory_mag(struct DOF* _joint, float _mag, float _period)
{
    trajectory[_joint->type].startTime = trajectory[_joint->type].startTime.now();
    trajectory[_joint->type].startPos = _joint->jpos;
    trajectory[_joint->type].startVel = _joint->jvel;
    _joint->jpos_d = _joint->jpos;
    _joint->jvel_d = _joint->jvel;

    trajectory[_joint->type].magnitude = _mag;
    trajectory[_joint->type].period = _period;
    return 0;
}

/**
*   \brief stop velocity trajectory()
*
*   Set jvel zero + zero torque
*
*   \param  _joint    DOF struct for specific joint
*   \ingroup Control
*
*/
int stop_trajectory(struct DOF* _joint)
{
    trajectory[_joint->type].startTime = trajectory[_joint->type].startTime.now();
    trajectory[_joint->type].startPos = _joint->jpos;
    trajectory[_joint->type].startVel = 0;
    _joint->jpos_d = _joint->jpos;
    _joint->jvel_d = 0;
    _joint->tau_d = 0;
    _joint->current_cmd = 0;

    return 0;
}

/**
*  update_sinusoid_trajectory()
*        find next trajectory waypoint
*        Sinusoid trajectory
*
*  \ingroup Control
*
* \todo This seems to only work for a single joint of the GOLD arm???
*/
int update_sinusoid_velocity_trajectory(struct DOF* _joint)
{
    const float maxspeed = 15 DEG2RAD;
    const float f_period = 2000;         // 2 sec

    ros::Duration t = ros::Time::now() - trajectory[_joint->type].startTime;

   if (_joint->type      == SHOULDER_GOLD)
        _joint->jvel_d = -1 * maxspeed * sin( 2*M_PI * (1/f_period) * t.toSec());

//    else if (_joint->type == ELBOW_GOLD)
//        _joint->jvel_d =  maxspeed * sin( 2*M_PI * (1/f_period) * t);
//
//    else if (_joint->type == Z_INS_GOLD)
//        _joint->jvel_d =  0.2 * sin( 2*M_PI * (1/f_period) * t);

    else
        _joint->jvel_d = 0;

    return 0;
}

/**
*
*  update_linear_sinusoid_trajectory()
*     find next trajectory waypoint.
*     Sinusoid ramp up and linear velocity after peak.
*     \ingroup Control
*
* \todo Why is this specific to the GOLD arm 1st three joints only?
*/
int update_linear_sinusoid_velocity_trajectory(struct DOF* _joint)
{
    const float maxspeed[8] = {-4 DEG2RAD, 4 DEG2RAD, 0.02, 15 DEG2RAD};
    const float f_period = 2;         // 2 sec

    ros::Duration t = ros::Time::now() - trajectory[_joint->type].startTime;

    // Sinusoid portion complete.  Return without changing velocity.
    if (t.toSec() >= f_period/2)
        return 1;

    if (_joint->type      == SHOULDER_GOLD)
        _joint->jvel_d = maxspeed[0] * (1-cos( 2*M_PI * (1/f_period) * t.toSec()));
	 

    else if (_joint->type == ELBOW_GOLD)
        _joint->jvel_d = maxspeed[1] * (1-cos( 2*M_PI * (1/f_period) * t.toSec()));

    else if (_joint->type == Z_INS_GOLD)
        _joint->jvel_d = maxspeed[2] * (1-cos( 2*M_PI * (1/f_period) * t.toSec()));

    else
        _joint->jvel_d = 0;

    return 0;
}

/**
*  update_sinusoid_position_trajectory()
*     find next trajectory waypoint.
*     Sinusoidal position trajectory
*     \ingroup Control
*
*   /todo What is the underlying equation?  Why piecewise at f_period/4??
*/
int update_sinusoid_position_trajectory(struct DOF* _joint)
{
    struct _trajectory* traj = &(trajectory[_joint->type]);
    float f_magnitude = traj->magnitude;
    float f_period    = traj->period;

    ros::Duration t = ros::Time::now() - traj->startTime;

    // Rising sinusoid
    if ( t.toSec() < f_period/4 )
        _joint->jpos_d = -f_magnitude * 0.5 * (1-cos( 4 * M_PI * t.toSec() / f_period )) + traj->startPos;
    else
        _joint->jpos_d = -f_magnitude * sin( 2*M_PI * t.toSec() / f_period) + traj->startPos;

	//std::cout <<"jpos_d" <<std::endl;
	//std::cout <<_joint->jpos_d <<std::endl;
    return 0;
}

/**
*  update_sinusoid_position_trajectory()
*     find next trajectory waypoint.
*     Sinusoidal position trajectory
*     \ingroup Control
*/
int update_linear_sinusoid_position_trajectory(struct DOF* _joint)
{
//    const float f_magnitude[8] = {-10 DEG2RAD, 10 DEG2RAD, 0.01, 0, 60 DEG2RAD, 60 DEG2RAD, 60 DEG2RAD, 60 DEG2RAD};
//    const float f_period[8] = {7000, 3200, 7000, 0000, 5000, 5000, 5000, 5000};
    struct _trajectory* traj = &(trajectory[_joint->type]);

    ros::Duration t = ros::Time::now() - traj->startTime;

    if ( t.toSec() < traj->period/2 )
//        _joint->jpos_d += ONE_MS * f_magnitude[index] * (1-cos( 2*M_PI * (1/f_period[index]) * t.toSec()));
        _joint->jpos_d += ONE_MS * traj->magnitude * (1-cos( 2*M_PI * (1/traj->period) * t.toSec()));
    else
        _joint->jpos_d += ONE_MS * traj->magnitude;

    return 0;
}


/**
*  update_sinusoid_position_trajectory()
*     find next trajectory waypoint.
*     Sinusoidal position trajectory
*     \ingroup Control
*/
int update_position_trajectory(struct DOF* _joint)
{
    struct _trajectory* traj = &(trajectory[_joint->type]);
    float magnitude = traj->magnitude;
    float period  = traj->period;

    ros::Duration t = ros::Time::now()- traj->startTime;

    if ( t.toSec() < period ){
        _joint->jpos_d = 0.5*magnitude * (1-cos( 2*M_PI * (1/(2*period)) * t.toSec())) + traj->startPos;
        return 1;
    }

    return 0;
}

int start_trajopt(struct DOF* _joint, double i){
     struct _trajectory* traj = &(trajectory[_joint->type]);
    _joint->jpos_d = Array[c][(int)i] + traj->startPos;
    //_joint->jpos_d = Array[c][(int)i];// 
    
    //std::cout<<_joint->jpos_d<< " " << Array[c][(int)i]<< " "<<c<<" "<<i<<std::endl;
    if (i==7 && c<4000){
    c++;
    
    //std::cout<<"incementing"<<std::endl;
    }
    else if(c==4000){
    c=0;
    }
    return 1;
}

int csv_read(){
 	    std::ifstream theFile ("/home/uva-dsa1/Downloads/trajopt/python_examples/traj_data_raven.csv");
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
                //std::cout<<Array[row][col]<<std::endl;  
                	col=col+1;
                }
                Array[row][col]=n;
                if (col==6){
                Array[row][col]-=M_PI/4;
                //std::cout<<Array[row][col]<<std::endl;        
                col=col+1;
                Array[row][col]+=M_PI/2;             
                Array[row][col]=-Array[row][col];   
                }
                //Array[row][col]=n;
        	//std::cout<<Array[row][col]<<std::endl;        
        	
            col=col+1;
            }
            
                    
            //std::cout<<"row"<<std::endl;            
            //std::cout<<row<<std::endl;
            row=row+1;;
            col=0;
            

            if (!lineStream && cell.empty())
            {
                // If there was a trailing comma then add an empty element.
                              //  std::cout <<" cell" << '\n';
                //m_data.push_back("");
            }
        }
        //ros::init(argc, argv, "listener");
  		//ros::NodeHandle n;
  		//ROS_INFO("subscribe");
        //ros::Subscriber sub = n.subscribe("send matrix", 1, matrixcb);
 return 1;
}

int subscriber_Read(float (&new_Array)[6]){
	std::cout<<"requirement met "<<reqCount<<std::endl; 
	reqCount++;
	isSub = true;
	bool debridement = false;
	int k=0;
	std::string s;
	subArray[6] = 0;
	if ((reqCount/20)%2 ==0){
		debridement = true;
	}
	if (prompt == true){
	
	for(int j=0	; j<8; j++){
		
		if (j==3){
		
		j=j+1;
		}
		subArray[j] = new_Array[k];		

		


		


		if (j==6){
		
		//subArray[i][j] -=M_PI/4;
		//ROS_INFO("data = %f\r\n",subArray[i][j]);
		j++;
		//subArray[i][j]+=M_PI/2;
		//subArray[i][j] = -subArray[i][j];
		if (debridement == false){
		subArray[j] = subArray[j-1] = M_PI/4;
		}
		else {
		subArray[j] = subArray[j-1] = 0;
		}
		}
		
		
		k++;
}
	std::cout<<"prompting done"<<std::endl;

	
}
prompt = false;
return 1;
}

int trajopt_with_node (struct DOF* _joint, double i){
	if ((int) i ==2 && isSub ==true){
		//subArray[(int) i] = 22.9035243988 DEG2RAD;
		std::cout<<"i=2: "<< subArray[(int) i]<<" ideal value: "<< 22.9 DEG2RAD<<std::endl;
		subArray[(int) i] = subArray[(int) i];

	}
     struct _trajectory* traj = &(trajectory[_joint->type]);
    _joint->jpos_d = subArray[(int)i];
   

    if(i==7){

    prompt = true;
    if (prompt ==true && isSub ==true){
    	std::cout<<"requirement met after "<<readCount<<std::endl;
    	readCount = 0;
    }
    if (prompt == true && isSub == false){
    	readCount++;
    }
    
        isSub=false;
    }
    return 1;

}


