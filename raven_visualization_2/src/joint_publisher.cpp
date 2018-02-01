/*
------------------Raven Joint Publisher Node------------------
--------------------Author: Sina Nia Kosari-------------------
-----------------------Date: 01-Dec-2011----------------------
----------------------Email: kosari@uw.edu--------------------
----------------------BioRobotics Laboratory------------------
--------------------University of Washington------------------
*/

//This Node Publishes Joint States for 2-Arm Visualization of Raven Robot.
//_L referes to the left arm (gold) and _R referes to the right arm (green).
//Change the value joint_state.position[n] to your desired joint states.


#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include "math.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "joint_publisher");
    ros::NodeHandle n;
    ros::Publisher joint_publisher = n.advertise<sensor_msgs::JointState>("joint_states", 1);
   
    //Publish 30 Messages/Sec
    ros::Rate loop_rate(30);
    double counter_inc = 1.0/30;
    double counter = 0.0;

    sensor_msgs::JointState joint_state;

    while (ros::ok()) {
        //update joint_state
        joint_state.header.stamp = ros::Time::now();
        joint_state.name.resize(14);
        joint_state.position.resize(14);

//======================LEFT ARM===========================

        joint_state.name[0] ="shoulder_L";
        joint_state.position[0] = -sin(.5*M_PI*counter+M_PI)-.5;

        joint_state.name[1] ="elbow_L";
        joint_state.position[1] = -.9*sin(.6*M_PI*counter+M_PI)-1.5;

        joint_state.name[2] ="insertion_L";
        joint_state.position[2] = .1*sin(.3*M_PI*counter);

        joint_state.name[3] ="tool_roll_L";
        joint_state.position[3] = sin(.4*M_PI*counter);

        joint_state.name[4] ="wrist_joint_L";
        joint_state.position[4] = sin(.5*M_PI*counter);

        joint_state.name[5] ="grasper_joint_1_L";
        joint_state.position[5] = .5*sin(M_PI*counter)+.5;

        joint_state.name[6] ="grasper_joint_2_L";
        joint_state.position[6] = -.5*sin(M_PI*counter)-.5;

//======================RIGHT ARM===========================

        joint_state.name[7] ="shoulder_R";
        joint_state.position[7] = -sin(.5*M_PI*counter)-.5;

        joint_state.name[8] ="elbow_R";
        joint_state.position[8] = .9*sin(.6*M_PI*counter)+1.5;

        joint_state.name[9] ="insertion_R";
        joint_state.position[9] = .1*sin(.3*M_PI*counter);

        joint_state.name[10] ="tool_roll_R";
        joint_state.position[10] = sin(.4*M_PI*counter);

        joint_state.name[11] ="wrist_joint_R";
        joint_state.position[11] = sin(.5*M_PI*counter);

        joint_state.name[12] ="grasper_joint_1_R";
        joint_state.position[12] = .5*sin(.5*M_PI*counter)+.5;

        joint_state.name[13] ="grasper_joint_2_R";
        joint_state.position[13] = -.5*sin(.5*M_PI*counter)-.5;

        //Publish the joint states
        joint_publisher.publish(joint_state);

        //increment timer
        counter += counter_inc;

        loop_rate.sleep();
    }


    return 0;
}

