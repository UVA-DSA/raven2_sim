//
//  main.cpp
//  LearnOMPL_2
//
//  Created by ming on 11/8/17.
//  Copyright © 2017 ming. All rights reserved.
//
#include <ompl/config.h>
#include <iostream>
#include <math.h>

#include "Kinematics/Kinematics.hpp"
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/SimpleSetup.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;

#ifdef forward_kinmatic
int main() {
    double in_j[6] = {0, 0, 0, 0, 0, 0}; // rad not degree
    tf::Transform end_pos; // I think these are in meter
    fwd_kin(in_j, dh_left, end_pos);
    std::cout << end_pos.getOrigin()[0] << " " << end_pos.getOrigin()[1] << " " << end_pos.getOrigin()[2] << std::endl;
    std::cout << end_pos.getRotation()[0] << " " << end_pos.getRotation()[1] << " "
              << end_pos.getRotation()[2] << " " << end_pos.getRotation()[3] << std::endl;
}
#endif

#ifdef planWithSimpleSetup
void planWithSimpleSetup()
{
    // construct the state space we are planning in
    auto space(std::make_shared<ob::RealVectorStateSpace(6));
    
    // set the bounds for the R^3 part of SE(3)
    ob::RealVectorBounds bounds(6);
    bounds.setLow(-1);
    bounds.setHigh(1);
    space->setBounds(bounds);
    
    // define a simple setup class
    og::SimpleSetup ss(space);
    
    // set state validity checking for this space
    ss.setStateValidityChecker([](const ob::State *state) { return isStateValid(state); });
    
    // create a random start state
    ob::ScopedState<> start(space);
    start.random();
    
    // create a random goal state
    ob::ScopedState<> goal(space);
    goal.random();
    
    // set the start and goal states
    ss.setStartAndGoalStates(start, goal);
    
    // this call is optional, but we put it in to get more output information
    ss.setup();
    ss.print();
    
    // attempt to solve the problem within one second of planning time
    ob::PlannerStatus solved = ss.solve(1.0);
    
    if (solved)
    {
        std::cout << "Found solution:" << std::endl;
        // print the path to screen
        ss.simplifySolution();
        ss.getSolutionPath().print(std::cout);
    }
    else
        std::cout << "No solution found" << std::endl;
}
#endif


