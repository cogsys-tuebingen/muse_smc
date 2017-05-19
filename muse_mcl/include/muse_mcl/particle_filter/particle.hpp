#ifndef PARTICLE_HPP
#define PARTICLE_HPP

#include <muse_mcl/math/pose.hpp>

#include <vector>

namespace muse_mcl {
struct Particle {
    using PoseType   = math::Pose;
    using WeightType = double;

    inline Particle() :
       pose_(tf::createQuaternionFromRPY(0,0,0),
             tf::Vector3(0,0,0)),
       weight_(1.0)
    {
    }

    inline Particle(const Particle &other) :
        pose_(other.pose_),
        weight_(other.weight_)
    {
    }

    inline Particle(Particle &&other) :
        pose_(std::move(other.pose_)),
        weight_(other.weight_)
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

    Particle& operator = (const Particle &other)
    {
        pose_ = other.pose_;
        weight_ = other.weight_;
        return *this;
    }

    Particle& operator = (Particle &&other)
    {
        pose_ = std::move(other.pose_);
        weight_ = other.weight_;
        return *this;
    }

    PoseType   pose_;
    WeightType weight_;

};
}

#endif /* PARTICLE */
