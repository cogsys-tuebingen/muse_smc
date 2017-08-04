#ifndef FILTER_HPP
#define FILTER_HPP

#include <muse_smc/prediction/prediction.hpp>
#include <muse_smc/update/update.hpp>
#include <muse_smc/sampling/sampling_normal.hpp>
#include <muse_smc/sampling/sampling_uniform.hpp>
#include <muse_smc/samples/sample_set.hpp>
#include <muse_smc/resampling/resampling.hpp>
#include <muse_smc/resampling/resampling_criterion.hpp>
#include <muse_smc/filter/state_publisher.hpp>


#include <memory>
#include <thread>
#include <atomic>
#include <queue>
#include <condition_variable>
#include <map>

namespace muse_smc {
template<typename sample_t>
class SMC
{
public:
    using Ptr                   = std::shared_ptr<SMC>;
    using sample_set_t          = SampleSet<sample_t>;
    using update_t              = Update<sample_t>;
    using prediction_t          = Prediction<sample_t>;
    using prediction_result_t   = prediction_t::predition_model_t::Result;
    using resampling_criterion_t= ResamplingCriterion<sample_t>;
    using normal_sampling_t     = SamplingNormal<sample_t>;
    using uniform_sampling_t    = SamplingUniform<sample_t>;
    using resampling_t          = Resampling<sample_t>;
    using state_publisher_t     = StatePublisher<sample_t>;
    using update_queue_t        = std::priority_queue<typename update_t::Ptr,
    std::deque<typename update_t::Ptr>,
    typename update_t::Greater>;

    using prediction_queue_t  = std::priority_queue<typename prediction_t::Ptr,
    std::deque<typename prediction_t::Ptr>,
    typename prediction_t::Greater>;

    SMC()
    {
    }

    virtual ~SMC()
    {
    }

    void setup(const sample_set_t::Ptr           &sample_set,
               const uniform_sampling_t::Ptr     &sample_uniform,
               const normal_sampling_t::Ptr      &sample_normal,
               const resampling_t::Ptr           &resampling,
               const resampling_criterion_t::Ptr &resampling_criterion,
               const state_publisher_t::Ptr      &state_publisher)
    {
        sample_set_          = sample_set;
        sample_uniform_      = sample_uniform;
        sample_normal_       = sample_normal;
        resampling_          = resampling;
        prediction_integral_ = resampling_criterion;
        state_publisher_     = state_publisher;
    }

    void start()
    {

    }

    void end()
    {

    }

    void addPrediction(const prediction_t::Ptr &prediction)
    {

    }

    void addUpdate(const update_t::Ptr &update)
    {

    }

    void requestNormalInitialization(const sample_t &sample,
                                     const typename sample_t::covariance_t)
    {

    }

    void requestUniformInitialization()
    {

    }

protected:
    sample_set_t::Ptr          sample_set_;
    uniform_sampling_t::Ptr    sample_uniform_;
    normal_sampling_t::Ptr     sample_normal_;
    resampling_t::Ptr          resampling_;
    resampling_criterion_t::Ptr prediction_integral_;
    state_publisher_t::Ptr     state_publisher_;

    void requests()
    {

    }

    void loop()
    {

    }

    void resample()
    {

    }

};
}

#endif // FILTER_HPP
