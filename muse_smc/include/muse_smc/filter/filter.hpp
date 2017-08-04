#ifndef FILTER_HPP
#define FILTER_HPP

#include <muse_smc/prediction/prediction.hpp>
#include <muse_smc/update/update.hpp>
#include <muse_smc/sampling/sampling_normal.hpp>
#include <muse_smc/sampling/sampling_uniform.hpp>
#include <muse_smc/samples/sample_set.hpp>
#include <muse_smc/resampling/resampling.hpp>
#include <muse_smc/resampling/resampling_criterion.hpp>
#include <muse_smc/filter/filter_state.hpp>


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
    using mutex_t               = std::mutex;
    using lock_t                = std::unique_lock<mutex_t>;

    /// filter specific type defs
    using sample_set_t          = SampleSet<sample_t>;
    using update_t              = Update<sample_t>;
    using prediction_t          = Prediction<sample_t>;
    using prediction_result_t   = prediction_t::predition_model_t::Result;
    using resampling_criterion_t= ResamplingCriterion<sample_t>;
    using normal_sampling_t     = SamplingNormal<sample_t>;
    using uniform_sampling_t    = SamplingUniform<sample_t>;
    using resampling_t          = Resampling<sample_t>;
    using filter_state_t        = FilterState<sample_t>;
    using update_queue_t        = std::priority_queue<typename update_t::Ptr,
    std::deque<typename update_t::Ptr>,
    typename update_t::Greater>;

    using prediction_queue_t  = std::priority_queue<typename prediction_t::Ptr,
    std::deque<typename prediction_t::Ptr>,
    typename prediction_t::Greater>;

    SMC() :
        request_init_state_(false),
        request_init_uniform_(false)
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
               const filter_state_t::Ptr          &state_publisher)
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

    void requestStateInitialization(const typename sample_t::state_t &state,
                                    const typename sample_t::covariance_t &covariance)
    {
        lock_t l(init_state_mutex_);
        init_state_ = state;
        init_state_covariance_;
        request_init_state_ = true;
    }

    void requestUniformInitialization()
    {
        request_init_uniform_ = true;
    }

protected:
    /// functions to apply to the sample set
    sample_set_t::Ptr           sample_set_;
    uniform_sampling_t::Ptr     sample_uniform_;
    normal_sampling_t::Ptr      sample_normal_;
    resampling_t::Ptr           resampling_;
    resampling_criterion_t::Ptr prediction_integral_;
    filter_state_t::Ptr         state_publisher_;

    /// requests
    std::mutex                      init_state_mutex_;
    typename sample_t::state_t      init_state_;
    typename sample_t::covariance_t init_state_covariance_;
    std::atomic_bool                request_init_state_;
    std::atomic_bool                request_init_uniform_;

    void requests()
    {
        if(request_init_state_) {
            sample_normal_->apply(init_state_,
                                  init_state_covariance_,
                                  sample_set_->getInsertion());
        }
        if(request_init_uniform_) {
            sample_uniform_->apply(sample_set_->getInsertion());
        }
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
