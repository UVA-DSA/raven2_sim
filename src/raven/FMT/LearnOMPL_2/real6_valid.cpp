//
//  real6_valid.cpp
//  LearnOMPL_2
//
//  Created by ming on 11/17/17.
//  Copyright © 2017 ming. All rights reserved.
//

#include "real6_valid.hpp"
#include <ompl/config.h>
#include <iostream>
#include <math.h>

#include "Kinematics/Kinematics.hpp"
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/fmt/FMT.h>
#include <ompl/geometric/planners/fmt/BFMT.h>
#include <ompl/geometric/SimpleSetup.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;


bool isStateValid(const ompl::base::State *state)
{
    /*
     // cast the abstract state type to the type we expect
     const auto *se3state = state->as<ompl::base::SE3StateSpace::StateType>();
     
     // extract the first component of the state and cast it to what we expect
     const auto *pos = se3state->as<ompl::base::RealVectorStateSpace::StateType>(0);
     
     // extract the second component of the state and cast it to what we expect
     const auto *rot = se3state->as<ompl::base::SO3StateSpace::StateType>(1);
     
     // check validity of state defined by pos & rot
     
     // return a value that is always true but uses the two variables we define, so we avoid compiler warnings
     */
    const double* joints = state->as<ob::RealVectorStateSpace::StateType>()->values;
    // Use forward kinematics
    static tf::Transform end_pos;
    fwd_kin((double *) joints, dh_left, end_pos);
    /*
    result->as<ob::SE3StateSpace::StateType>()->setXYZ(end_pos.getOrigin()[0], end_pos.getOrigin()[1], end_pos.getOrigin()[2]);
    tf::Quaternion temp_qu = end_pos.getRotation();
    result->as<ob::SO3StateSpace::StateType>()->setAxisAngle(temp_qu[0], temp_qu[1], temp_qu[2], temp_qu[3]);
     */
    
    double x = end_pos.getOrigin()[0]; double y = end_pos.getOrigin()[1]; double z = end_pos.getOrigin()[1];
    /*
    if ((x > 0.5 && x < 1) && (y > 0.5 && y <1) && (x > 0.5 && z < 1)) {
        return 0;
    }*/
    /*
    if (joints[2]>0.2 && joints[0]<0.5) {
        return 0;
    } */
    return 1;
}

void plan()
{
    /* Only setBounds to ScopedState, not StateSpace
     * To use a new space, first create a CompoundStateSpace, then inherit it. */
    
    ob::CompoundStateSpace *newSpace = new ob::CompoundStateSpace();
    newSpace->addSubspace(ob::StateSpacePtr(new ob::RealVectorStateSpace(6)), 1.0);
    
    ob::StateSpacePtr space(newSpace);
    
    
    // set the bounds for the R^3 part of SE(3)
    //ompl::base::RealVectorBounds bounds(6);
    //bounds.setLow(-5);
    //bounds.setHigh(5);
    space->as<ob::CompoundStateSpace>()->as<ob::RealVectorStateSpace>(0)->setBounds(-2, 2);
    
    // construct an instance of  space information from this state space
    auto si(std::make_shared<ob::SpaceInformation>(space));
    
    // set state validity checking for this space
    si->setStateValidityChecker(isStateValid);
    
    // create a random start state
    ob::ScopedState<> start(space);
    start[0] = 0; start[1] = 0; start[2] = 0; start[3] = 0; start[4] = 0; start[5] = 0;
    //std::cout << start;
    
    // create a random goal state
    ob::ScopedState<> goal(space);
    goal[0] = 0; goal[1] = 0; goal[2] = 2;// goal[3] = 3; goal[4] = 3; goal[5] = 3;
    //std::cout << goal;
    
    // create a problem instance
    auto pdef(std::make_shared<ompl::base::ProblemDefinition>(si));
    
    // set the start and goal states
    pdef->setStartAndGoalStates(start, goal, 0.5);
    
    // create a planner for the defined space
    auto planner(new og::RRTConnect(si));
    //auto planner(new og::FMT(si));
    //auto planner(new og::BFMT(si));
    //auto planner(new og::planner RRTConnect(si));
    //auto planner(new og::RRTConnect(si));
    
    // set the problem we are trying to solve for the planner
    planner->setProblemDefinition(pdef);
    
    // perform setup steps for the planner
    planner->setup();
    
    
    // print the settings for this space
    si->printSettings(std::cout);
    
    // print the problem settings
    pdef->print(std::cout);
    
    // attempt to solve the problem within one second of planning time
    ompl::base::PlannerStatus solved = planner->ompl::base::Planner::solve(10.0);
    
    if (solved)
    {
        // get the goal representation from the problem definition (not the same as the goal state)
        // and inquire about the found path
        ompl::base::PathPtr path = pdef->getSolutionPath();
        std::cout << "Found solution:" << std::endl;
        
        // print the path to screen
        path->print(std::cout);
        //pdef->getSolutionPath().printAsMatrix(std::cout);
    }
    else
        std::cout << "No solution found" << std::endl;
}

int main(int /*argc*/, char ** /*argv*/)
{
    std::cout << "OMPL version: " << OMPL_VERSION << std::endl;
    plan();
    return 0;
}
