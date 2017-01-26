#ifndef PARTICLE_FILTER_HPP
#define PARTICLE_FILTER_HPP

#include "particle_set.hpp"

#include "../data_sources/tf_provider.hpp"
#include "resampling.hpp"
#include "sampling_uniform.hpp"
#include "sampling_normal.hpp"

#include "prediction.hpp"
#include "update.hpp"

#include <memory>
#include <thread>
#include <atomic>


namespace muse_amcl {
class ParticleFilter {
public:
    using Ptr = std::shared_ptr<ParticleFilter>;

    ParticleFilter() :
        name_("particle_filter")
    {

    }

    void setup(ros::NodeHandle &nh_private,
               const TFProvider::Ptr &tf_provider)
    {
        tf_provider_ = tf_provider;
    }

    /// uniform pose sampling
    void setUniformSampling(UniformSampling::Ptr &uniform_sampling)
    {
        uniform_sampling = uniform_sampling;

    }
    UniformSampling::Ptr getUniformSampling() const
    {
        return uniform_sampling_;
    }

    /// normal pose sampling
    void setNormalsampling(NormalSampling::Ptr &normal_sampling)
    {
        normal_sampling_ = normal_sampling;
    }
    NormalSampling::Ptr getNormalSampling() const
    {
        return normal_sampling_;
    }

    /// resampling
    void setResampling(Resampling::Ptr &resampling)
    {
        resampling_ = resampling;
    }
    Resampling::Ptr getResampling() const
    {
        return resampling_;
    }

    /// insert new predictions
    void addPrediction(Prediction::Ptr &prediction)
    {

    }

    void addUpdate(Update::Ptr &update)
    {

    }


protected:
    std::string         name_;
    TFProvider::Ptr     tf_provider_;
    ParticleSet::Ptr    particle_set_;

    UniformSampling::Ptr  uniform_sampling_;
    NormalSampling::Ptr   normal_sampling_;
    Resampling::Ptr       resampling_;






    std::string parameter(const std::string &name)
    {
        return name_ + "/" + name;
    }


};
}

#endif // PARTICLE_FILTER_HPP
