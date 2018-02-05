#ifndef RESAMPLING_HPP
#define RESAMPLING_HPP

#include <muse_smc/samples/sample_set.hpp>
#include <muse_smc/sampling/uniform.hpp>
#include <muse_smc/sampling/normal.hpp>
#include <muse_smc/prediction/prediction_integral.hpp>

#include <cslibs_math/random/random.hpp>

#include <memory>
#include <vector>
#include <iostream>

namespace muse_smc {
template<typename state_space_description_t>
class Resampling
{
public:
    using Ptr = std::shared_ptr<Resampling>;
    using sample_t              = typename state_space_description_t::sample_t;
    using sample_set_t          = SampleSet<state_space_description_t>;
    using sample_uniform_t      = UniformSampling<state_space_description_t>;
    using sample_normal_t       = NormalSampling<state_space_description_t>;
    using prediction_integral_t = PredictionIntegral<state_space_description_t>;

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
        return "muse_smc::Resampling";
    }

    inline void setName(const std::string &name)
    {
        name_ = name;
    }

    inline std::string getName() const
    {
        return name_;
    }

    inline std::size_t getId() const
    {
        return id_;
    }

    inline void setId(const std::size_t id)
    {
        id_ = id;
    }

    virtual inline void setup(const typename sample_uniform_t::Ptr &uniform_pose_sampler,
                              const typename sample_normal_t::Ptr  &normal_pose_sampler,
                              const double                          recovery_alpha_fast = 0.0,
                              const double                          recovery_alpha_slow = 0.0)
    {
        uniform_pose_sampler_ = uniform_pose_sampler;
        normal_pose_sampler_ = normal_pose_sampler;
        recovery_alpha_fast_  = recovery_alpha_fast;
        recovery_alpha_slow_  = recovery_alpha_slow;
    }

    inline void apply(sample_set_t &sample_set)
    {
        if(sample_set.getWeightSum() == 0.0) {
            std::cerr << "[MuseSMC]: All particle weights are zero. \n";
        }
        updateRecovery(sample_set);
        auto do_apply = [&sample_set, this] () {
            doApply(sample_set);
        };
        auto do_apply_recovery = [&sample_set, this] () {
            doApplyRecovery(sample_set);
        };
        return recovery_random_pose_probability_ == 0.0 ? do_apply() : do_apply_recovery();
    }

    inline void resetRecovery()
    {
        recovery_fast_ = 0.0;
        recovery_slow_ = 0.0;
    }

protected:
    std::string                     name_;
    std::size_t                     id_;
    double                          recovery_alpha_fast_;
    double                          recovery_alpha_slow_;
    double                          recovery_fast_;
    double                          recovery_slow_;
    double                          recovery_random_pose_probability_;
    typename sample_uniform_t::Ptr  uniform_pose_sampler_;
    typename sample_normal_t::Ptr   normal_pose_sampler_;

    virtual void doApply(sample_set_t &sample_set) = 0;
    virtual void doApplyRecovery(sample_set_t &sample_set) = 0;

    inline void updateRecovery(sample_set_t &particle_set)
    {
        const double weight_average = particle_set.getAverageWeight();
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

        recovery_random_pose_probability_ = 1.0 - recovery_fast_ / recovery_slow_;
    }
};
}

#endif /* RESAMPLING_HPP */
