#ifndef PARTICLE_HPP
#define PARTICLE_HPP

#include <muse_amcl/math/pose.hpp>

namespace muse_amcl {
struct Particle {
    using PoseType   = math::Pose;
    using WeightType = double;

    Particle() :
       pose_(tf::createQuaternionFromRPY(0,0,0),
            tf::Vector3(0,0,0)),
       weight_(0.0)
    {
    }

    Particle(const tf::Quaternion &q,
             const tf::Vector3    &p,
             const WeightType     &w) :
        pose_(q, p),
        weight_(w)
    {
    }

    Particle(const tf::Pose &p,
             const WeightType w)  :
        pose_(p),
        weight_(w)
    {
    }

    Particle(const PoseType::Vector3d &p,
             const WeightType w) :
        pose_(p),
        weight_(w)
    {
    }

    Particle(const PoseType::Vector6d &p,
             const WeightType w) :
        pose_(p),
        weight_(w)
    {
    }

    PoseType   pose_;
    WeightType weight_;

};
}

#endif /* PARTICLE */
