#ifndef RESAMPLING_HPP
#define RESAMPLING_HPP

#include <memory>
#include <vector>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <ros/node_handle.h>

#include "particle_set.hpp"
#include "sampling_uniform.hpp"
#include "../math/random.hpp"

namespace muse_amcl {
class Resampling {
public:
    typedef std::shared_ptr<Resampling> Ptr;

    Resampling() :
        recovery_alpha_fast_(0.0),
        recovery_alpha_slow_(0.0),
        recovery_fast_(0.0),
        recovery_slow_(0.0)
    {
    }

    virtual ~Resampling()
    {
    }

    inline const static std::string Type()
    {
        return "muse_amcl::PoseGeneration";
    }

    inline std::string getName() const
    {
        return name_;
    }

    inline void setup(const std::string          &name,
                      ros::NodeHandle            &nh_private,
                      const UniformSampling::Ptr &uniform_pose_sampler)
    {
        name_ = name;
        uniform_pose_sampler_ = uniform_pose_sampler;
        recovery_alpha_fast_ = nh_private.param(parameter("recovery_alpha_fast"), 0.0);
        recovery_alpha_slow_ = nh_private.param(parameter("recovery_alpha_slow"), 0.0);
    }

    inline void apply(ParticleSet &particle_set)
    {
        updateRecovery(particle_set);
        if(recovery_random_pose_probability_ == 0.0)
            doApply(particle_set);
        else
            doApplyRecovery(particle_set);
    }

    inline void resetRecovery()
    {
        recovery_fast_ = 0.0;
        recovery_slow_ = 0.0;
    }

protected:
    std::string                   name_;
    double                        recovery_alpha_fast_;
    double                        recovery_alpha_slow_;
    double                        recovery_fast_;
    double                        recovery_slow_;
    double                        recovery_random_pose_probability_;
    UniformSampling::Ptr          uniform_pose_sampler_;

    virtual void doSetup(ros::NodeHandle &nh_private) = 0;
    virtual void doApply(ParticleSet &particle_set) = 0;
    virtual void doApplyRecovery(ParticleSet &particle_set) = 0;

    std::string parameter(const std::string &name)
    {
        return name_ + "/" + name;
    }

    inline void updateRecovery(ParticleSet &particle_set)
    {
        const double weight_average = particle_set.getWeightAverage();
        if(recovery_slow_ == 0.0) {
            recovery_slow_ = weight_average;
        } else {
            recovery_slow_ += recovery_alpha_slow_ * (weight_average - recovery_slow_);
        }
        if(recovery_fast_ == 0.0) {
            recovery_fast_ = weight_average;
        } else {
            recovery_fast_ += recovery_alpha_fast_ * (weight_average - recovery_fast_);
        }

        recovery_random_pose_probability_ = 1.0 - recovery_alpha_fast_ / recovery_alpha_slow_;
    }
};
}

#endif /* RESAMPLING_HPP */
