//
//  isValid.cpp
//  LearnOMPL_2
//
//  Created by ming on 11/17/17.
//  Copyright © 2017 ming. All rights reserved.
//

#include "isValid.hpp"

#ifdef isValid
using Vector3 = fcl::Vector3d;
using Quaternion = fcl::Quaterniond;
using Transform = fcl::Transform3d;
using BVType = fcl::OBBRSS<double>;
using Model = fcl::BVHModel<BVType>;
using MeshDistanceTraversalNodeOBBRSS = fcl::detail::MeshDistanceTraversalNodeOBBRSS<double>;
using CollisionRequest = fcl::CollisionRequest<double>;
using CollisionResult = fcl::CollisionResult<double>;
using ContinuousCollisionRequest = fcl::ContinuousCollisionRequest<double>;
using ContinuousCollisionResult = fcl::ContinuousCollisionResult<double>;
using DistanceRequest = fcl::DistanceRequest<double>;
using DistanceResult = fcl::DistanceResult<double>;


/// \brief Checks whether the given robot state collides with the
/// environment or itself.
virtual bool isValid(const base::State *state) const
{
    static Transform identity(Transform::Identity());
    CollisionRequest collisionRequest;
    CollisionResult collisionResult;
    Transform transform;
    
    if (environment_.num_tris > 0)
    {
        // Performing collision checking with environment.
        for (std::size_t i = 0; i < robotParts_.size(); ++i)
        {
            poseFromStateCallback_(transform, extractState_(state, i));
            if (fcl::collide(robotParts_[i], transform, &environment_,
                             identity, collisionRequest, collisionResult) > 0)
                return false;
        }
    }
    
    // Checking for self collision
    if (selfCollision_)
    {
        Transform trans_i, trans_j;
        for (std::size_t i = 0 ; i < robotParts_.size(); ++i)
        {
            poseFromStateCallback_(trans_i, extractState_(state, i));
            
            for (std::size_t j  = i + 1 ; j < robotParts_.size(); ++j)
            {
                poseFromStateCallback_(trans_j, extractState_(state, j));
                if (fcl::collide(robotParts_[i], trans_i, robotParts_[j], trans_j,
                                 collisionRequest, collisionResult) > 0)
                    return false;
            }
        }
    }
    
    return true;
}
#endif
