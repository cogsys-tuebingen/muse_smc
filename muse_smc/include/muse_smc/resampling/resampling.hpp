#ifndef RESAMPLING_HPP
#define RESAMPLING_HPP

#include <muse_smc/samples/sample_set.hpp>
#include <muse_smc/sampling/uniform.hpp>
#include <muse_smc/prediction/prediction_integral.hpp>

#include <muse_smc/math/random.hpp>

#include <memory>
#include <vector>

namespace muse_smc {
template<typename sample_t>
class Resampling
{
public:
    using Ptr = std::shared_ptr<Resampling>;
    using sample_set_t = SampleSet<sample_t>;
    using sample_uniform_t = UniformSampling<sample_t>;
    using prediction_integral_t = PredictionIntegral<sample_t>;

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
                              const double                          recovery_alpha_fast = 0.0,
                              const double                          recovery_alpha_slow = 0.0)
    {
        uniform_pose_sampler_ = uniform_pose_sampler;
        recovery_alpha_fast_  = recovery_alpha_fast;
        recovery_alpha_slow_  = recovery_alpha_slow;
    }

    inline void apply(sample_set_t &sample_set)
    {
        if(sample_set.getWeightSum() == 0.0) {
            std::cerr << "[MuseSMC]: Cannot resample if all weights are zero! \n";
            return;
        }

        updateRecovery(sample_set);
        if(recovery_random_pose_probability_ == 0.0)
            doApply(sample_set);
        else
            doApplyRecovery(sample_set);
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
