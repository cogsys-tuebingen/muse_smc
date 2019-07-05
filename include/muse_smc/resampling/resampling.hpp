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
template<typename sample_t>
class Resampling
{
public:
    using Ptr                   = std::shared_ptr<Resampling>;
    using sample_set_t          = SampleSet<sample_t>;
    using sample_uniform_t      = UniformSampling<sample_t>;
    using sample_normal_t       = NormalSampling<sample_t>;

    Resampling() :
        recovery_alpha_fast_(0.0),
        recovery_alpha_slow_(0.0),
        recovery_fast_(0.0),
        recovery_slow_(0.0),
        variance_treshold_(0.0)
    {
    }

    virtual ~Resampling() = default;

    virtual inline void setup(const typename sample_uniform_t::Ptr &uniform_pose_sampler,
                              const typename sample_normal_t::Ptr  &normal_pose_sampler,
                              const double                          recovery_alpha_fast = 0.0,
                              const double                          recovery_alpha_slow = 0.0,
                              const double                          variance_threshold = 0.0)
    {
        uniform_pose_sampler_ = uniform_pose_sampler;
        normal_pose_sampler_  = normal_pose_sampler;
        recovery_alpha_fast_  = recovery_alpha_fast;
        recovery_alpha_slow_  = recovery_alpha_slow;
        variance_treshold_    = variance_threshold;
    }

    inline void apply(sample_set_t &sample_set)
    {
        if (sample_set.getWeightSum() == 0.0) {
            std::cerr << "[MuseSMC]: All particle weights are zero. \n";
            return;
        }

        if(sample_set.getWeightVariance() < variance_treshold_) {
            std::cerr << "[MuseSMC]: Variance not high enough for resampling. \n";
            return;
        }


        auto do_apply = [&sample_set, this] () {
            doApply(sample_set);
        };
        auto do_apply_recovery = [&sample_set, this] () {
            doApplyRecovery(sample_set);
            resetRecovery();
        };

        recovery_random_pose_probability_ == 0.0 ? do_apply() : do_apply_recovery();
    }

    inline void resetRecovery()
    {
        recovery_fast_ = 0.0;
        recovery_slow_ = 0.0;
    }

    inline void updateRecovery(sample_set_t &particle_set)
    {
        const double weight_average = particle_set.getAverageWeight();
        if (recovery_slow_ == 0.0)
            recovery_slow_ = weight_average;
        else
            recovery_slow_ += recovery_alpha_slow_ * (weight_average - recovery_slow_);

        if (recovery_fast_ == 0.0)
            recovery_fast_ = weight_average;
        else
            recovery_fast_ += recovery_alpha_fast_ * (weight_average - recovery_fast_);

        if (recovery_slow_ != 0.0)
            recovery_random_pose_probability_ = std::max(0.0, 1.0 - recovery_fast_ / recovery_slow_);
    }

protected:
    double                          recovery_alpha_fast_;
    double                          recovery_alpha_slow_;
    double                          recovery_fast_;
    double                          recovery_slow_;
    double                          recovery_random_pose_probability_;
    double                          variance_treshold_;
    typename sample_uniform_t::Ptr  uniform_pose_sampler_;
    typename sample_normal_t::Ptr   normal_pose_sampler_;

    virtual void doApply(sample_set_t &sample_set) = 0;
    virtual void doApplyRecovery(sample_set_t &sample_set) = 0;
};
}

#endif // RESAMPLING_HPP
