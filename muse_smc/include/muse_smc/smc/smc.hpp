#ifndef FILTER_HPP
#define FILTER_HPP

#include <muse_smc/prediction/prediction.hpp>
#include <muse_smc/update/update.hpp>
#include <muse_smc/sampling/normal.hpp>
#include <muse_smc/sampling/uniform.hpp>
#include <muse_smc/samples/sample_set.hpp>
#include <muse_smc/resampling/resampling.hpp>
#include <muse_smc/smc/smc_state.hpp>
#include <muse_smc/utility/synchronized_priority_queue.hpp>
#include <muse_smc/utility/dotty.hpp>
#include <muse_smc/time/rate.hpp>

#include <memory>
#include <thread>
#include <atomic>
#include <queue>
#include <condition_variable>
#include <map>

/***
 * Distance thresholds for resampling and update throttling are
 * currently left out of the game. Questionable is, if time
 * thresholds respectively rates can be used instead.
 */


namespace muse_smc {
template<typename sample_t>
class SMC
{
public:
    using Ptr                   = std::shared_ptr<SMC>;
    using mutex_t               = std::mutex;
    using condition_variable_t = std::condition_variable;
    using lock_t                = std::unique_lock<mutex_t>;
    using thread_t              = std::thread;
    using atomic_bool_t         = std::atomic_bool;

    /// filter specific type defs
    using sample_set_t          = SampleSet<sample_t>;
    using update_t              = Update<sample_t>;
    using prediction_t          = Prediction<sample_t>;
    using prediction_result_t   = typename prediction_t::predition_model_t::Result;
    using prediction_integral_t = PredictionIntegral<sample_t>;
    using normal_sampling_t     = NormalSampling<sample_t>;
    using uniform_sampling_t    = UniformSampling<sample_t>;
    using resampling_t          = Resampling<sample_t>;
    using filter_state_t        = SMCState<sample_t>;
    using update_queue_t        =
    muse_smc::synchronized::priority_queue<typename update_t::Ptr,
    std::deque<typename update_t::Ptr>,
    typename update_t::Greater>;

    using prediction_queue_t  =
    muse_smc::synchronized::priority_queue<typename prediction_t::Ptr,
    std::deque<typename prediction_t::Ptr>,
    typename prediction_t::Greater>;

    SMC() :
        updates_applied_after_resampling_(0ul),
        request_init_state_(false),
        request_init_uniform_(false),
        worker_thread_active_(false),
        worker_thread_exit_(false)
    {
    }

    virtual ~SMC()
    {
        end();
    }

    void setup(const typename sample_set_t::Ptr            &sample_set,
               const typename uniform_sampling_t::Ptr      &sample_uniform,
               const typename normal_sampling_t::Ptr       &sample_normal,
               const typename resampling_t::Ptr            &resampling,
               const typename filter_state_t::Ptr          &state_publisher,
               const Rate                                  &preferred_filter_state_update_rate)
    {
        sample_set_          = sample_set;
        sample_uniform_      = sample_uniform;
        sample_normal_       = sample_normal;
        resampling_          = resampling;
        state_publisher_     = state_publisher;

        prediction_integral_update_.reset(new prediction_integral_t);
        prediction_integral_resampling_.reset(new prediction_integral_t);
#ifdef USE_DOTTY
        dotty_.reset(new Dotty);
#endif

    }

    bool start()
    {
        if(!worker_thread_active_) {
            lock_t l(worker_thread_mutex_);
            worker_thread_exit_ = false;
            worker_thread_ = thread_t([this](){loop();});
            worker_thread_.detach();
            return true;
        }
        return false;
    }

    bool end()
    {
        if(!worker_thread_active_)
            return false;

        worker_thread_exit_ = true;
        notify_event_.notify_one();
        if(worker_thread_.joinable()) {
            worker_thread_.join();
        }
    }

    void addPrediction(const typename prediction_t::Ptr &prediction)
    {
        prediction_queue_.emplace(prediction);
    }

    void addUpdate(const typename update_t::Ptr &update)
    {
        update_queue_.emplace(update);
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
    typename sample_set_t::Ptr           sample_set_;
    typename uniform_sampling_t::Ptr     sample_uniform_;
    typename normal_sampling_t::Ptr      sample_normal_;
    typename resampling_t::Ptr           resampling_;
    typename prediction_integral_t::Ptr  prediction_integral_update_;

    typename prediction_integral_t::Ptr  prediction_integral_resampling_;
    std::size_t                          updates_applied_after_resampling_;


    typename filter_state_t::Ptr         state_publisher_;

    /// requests
    std::mutex                          init_state_mutex_;
    typename sample_t::state_t          init_state_;
    typename sample_t::covariance_t     init_state_covariance_;
    atomic_bool_t                       request_init_state_;
    atomic_bool_t                       request_init_uniform_;

    /// processing queues
    update_queue_t                      update_queue_;
    prediction_queue_t                  prediction_queue_;

    /// background thread
    mutex_t                             worker_thread_mutex_;
    thread_t                            worker_thread_;
    atomic_bool_t                       worker_thread_active_;
    atomic_bool_t                       worker_thread_exit_;
    condition_variable_t                notify_event_;
    mutable mutex_t                     notify_event_mutex_;
    condition_variable_t                notify_prediction_;
    mutable mutex_t                     notify_prediction_mutex_;

    /// debugging
#ifdef USE_DOTTY
    Dotty::Ptr                          dotty_;
#endif

    bool hasUpdates()
    {
        return !update_queue_.empty();
    }

    bool hasPredictions()
    {
        return !prediction_queue_.empty();
    }

    void requests()
    {
        if(request_init_state_) {
            sample_normal_->apply(init_state_,
                                  init_state_covariance_,
                                  sample_set_->getInsertion());
            state_publisher_->publishIntermidiate(sample_set_);
        }
        if(request_init_uniform_) {
            sample_uniform_->apply(sample_set_->getInsertion());
            state_publisher_->publishIntermidiate(sample_set_);
        }
    }

    void predict(const Time &until)
    {
        auto wait_for_prediction = [this] () {
            if(prediction_queue_.empty()) {
                lock_t l(notify_prediction_mutex_);
                notify_prediction_.wait(l);
            }
        };

        const Time &time_stamp = sample_set_->getStamp();
        while(until > time_stamp) {
            wait_for_prediction();
            typename prediction_t::Ptr prediction = prediction_queue_.pop();
            if(prediction->getStamp() < time_stamp) {
                /// drop odometry messages which are too old
                continue;
            }

            /*
             * There must be more logic behind this.
             * -> we have to check if motion was fully applied
             * +-> if so check if the timestamp is equal to 'until'
             * -> split and retries have to be handled.
             */

            /// mutate time stamp
            typename prediction_result_t::Ptr prediction_result = prediction->apply(until, sample_set_->getStateIterator());
            if(prediction_result.success()) {
                prediction_integral_update_->add(prediction_result);
                prediction_integral_resampling_->add(prediction_result);
                sample_set_->setStamp(prediction_result->applied->getTimeFrame().end);

#ifdef USE_DOTTY
                dotty_->addPrediction(prediction_result->applied->getTimeFrame().end, static_cast<bool>(prediction_result->left_to_apply));
#endif

                if(prediction_result->left_to_apply) {
                    typename prediction_t::Ptr prediction_left_to_apply
                            (new prediction_t(prediction_result->left_to_apply, prediction->getModel()));
                    prediction_queue_.emplace(prediction_left_to_apply);
                    break;
                }
            } else {
                prediction_queue_.emplace(prediction);
            }
        }
    }

    void loop()
    {
        worker_thread_active_ = true;
        lock_t notify_event_mutex_lock(notify_event_mutex_);

        while(!worker_thread_exit_) {
            notify_event_.wait(notify_event_mutex_lock);

            if(worker_thread_exit_)
                break;

            if(sample_set_->getSampleSize() == 0) {
                sample_uniform_->apply(*sample_set_);
            }

            while(update_queue_.hasElements()) {
                if(worker_thread_exit_)
                    break;

                requests(); /// process all request that came in

                typename update_t::Ptr u = update_queue_.pop();
                const Time &t = u->getStamp();
                const Time &sample_set_stamp = sample_set_->getStamp();

                if(t >= sample_set_stamp) {
                    predict(t);
                    if(t > sample_set_stamp) {
                        update_queue_.emplace(u);
                    } else if (t == sample_set_stamp) {
                        if(!prediction_integral_update_->isZero()) {
                            u->apply(sample_set_->getWeightIterator());
                            prediction_integral_update_->reset();
                            ++updates_applied_after_resampling_;
#ifdef USE_DOTTY
                            dotty_->addState(sample_set_stamp);
                            dotty_->addUpdate(u->getStamp(), u->getModelName());
#endif
                        }
                    } else {
                        std::cerr << "Motion model seems not to be able to interpolate!" << std::endl;
                    }
                }
                if(!prediction_integral_resampling_->isZero() &&
                        updates_applied_after_resampling_ > 0ul) {
                    resampling_->apply(sample_set_);
                    updates_applied_after_resampling_ = 0ul;

                    state_publisher_->publish(sample_set_);
                } else {
                    state_publisher_->publishIntermidiate(sample_set_);
                }
            }
        }
        worker_thread_active_ = false;
    }
};
}

#endif // FILTER_HPP
