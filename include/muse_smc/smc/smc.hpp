#ifndef FILTER_HPP
#define FILTER_HPP

#include <muse_smc/prediction/prediction_integrals.hpp>
#include <muse_smc/prediction/prediction.hpp>
#include <muse_smc/update/update.hpp>
#include <muse_smc/sampling/normal.hpp>
#include <muse_smc/sampling/uniform.hpp>
#include <muse_smc/samples/sample_set.hpp>
#include <muse_smc/resampling/resampling.hpp>
#include <muse_smc/smc/smc_state.hpp>
#include <muse_smc/scheduling/scheduler.hpp>
#include <cslibs_time/rate.hpp>
#include <cslibs_time/statistics/duration_lowpass.hpp>
#include <cslibs_utility/synchronized/synchronized_priority_queue.hpp>


#ifdef MUSE_SMC_USE_DOTTY
#include <muse_smc/utility/dotty.hpp>
#endif
#ifdef MUSE_SMC_LOG_STATE
#include <muse_smc/utility/csv_logger.hpp>
#endif
#ifdef MUSE_SMC_DEBUG
#include <cslibs_math/statistics/mean.hpp>
#endif

#include <memory>
#include <thread>
#include <atomic>
#include <queue>
#include <condition_variable>
#include <unordered_map>

/***
 * Distance thresholds for resampling and update throttling are
 * currently left out of the game. Questionable is, if time
 * thresholds respectively rates can be used instead.
 */

namespace muse_smc {
template<typename state_space_description_t, typename data_t>
class SMC
{
public:
    using Ptr                   = std::shared_ptr<SMC>;
    using mutex_t               = std::mutex;
    using condition_variable_t  = std::condition_variable;
    using lock_t                = std::unique_lock<mutex_t>;
    using thread_t              = std::thread;
    using atomic_bool_t         = std::atomic_bool;

    /// filter specific type defs
    using sample_t              = typename state_space_description_t::sample_t;
    using sample_set_t          = SampleSet<state_space_description_t>;
    using state_t               = typename state_space_description_t::state_t;
    using covariance_t          = typename state_space_description_t::covariance_t;
    using update_t              = Update<state_space_description_t, data_t>;
    using prediction_t          = Prediction<state_space_description_t, data_t>;
    using prediction_result_t   = typename prediction_t::predition_model_t::Result;
    using prediction_integral_t = PredictionIntegral<state_space_description_t, data_t>;
    using prediction_integrals_t= PredictionIntegrals<state_space_description_t, data_t>;
    using normal_sampling_t     = NormalSampling<state_space_description_t>;
    using uniform_sampling_t    = UniformSampling<state_space_description_t>;
    using resampling_t          = Resampling<state_space_description_t, data_t>;
    using scheduler_t           = Scheduler<state_space_description_t, data_t>;
    using filter_state_t        = SMCState<state_space_description_t>;
    using update_queue_t        = cslibs_utility::synchronized::priority_queue<typename update_t::Ptr,
                                                                               typename update_t::Greater>;
    using prediction_queue_t    = cslibs_utility::synchronized::priority_queue<typename prediction_t::Ptr,
                                                                               typename prediction_t::Greater>;
    using duration_t            = cslibs_time::Duration;
    using duration_map_t        = std::unordered_map<std::size_t, cslibs_time::statistics::DurationLowpass>;

    inline SMC() :
        request_init_state_(false),
        request_init_uniform_(false),
        request_update_uniform_(false),
        worker_thread_active_(false),
        worker_thread_exit_(false)
    {
    }

    virtual ~SMC()
    {
        end();
    }

    inline void setup(const typename sample_set_t::Ptr            &sample_set,
                      const typename uniform_sampling_t::Ptr      &sample_uniform,
                      const typename normal_sampling_t::Ptr       &sample_normal,
                      const typename resampling_t::Ptr            &resampling,
                      const typename filter_state_t::Ptr          &state_publisher,
                      const typename prediction_integrals_t::Ptr  &prediction_integrals,
                      const typename scheduler_t::Ptr             &scheduler,
                      const bool                                  &enable_lag_correction)
    {
        sample_set_             = sample_set;
        sample_uniform_         = sample_uniform;
        sample_normal_          = sample_normal;
        resampling_             = resampling;
        state_publisher_        = state_publisher;
        prediction_integrals_   = prediction_integrals;
        scheduler_              = scheduler;
        enable_lag_correction_  = enable_lag_correction;

#ifdef MUSE_SMC_USE_DOTTY
        dotty_.reset(new Dotty);
#endif
#ifdef MUSE_SMC_LOG_STATE
        SMCFilterStateLogger::Header log_header = {"predictions, updates, time_ratio"};
        log_.reset(new SMCFilterStateLogger(log_header));
#endif
    }

    inline bool start()
    {
        if(!worker_thread_active_) {
            lock_t l(worker_thread_mutex_);
            worker_thread_exit_ = false;
            worker_thread_      = thread_t([this](){loop();});
#ifdef MUSE_SMC_DEBUG
        std::cerr << "Started filter core! \n";
#endif
            return true;
        }
        return false;
    }

    inline bool end()
    {
        if(!worker_thread_active_)
            return false;

        worker_thread_exit_ = true;
        notify_event_.notify_one();
        if(worker_thread_.joinable()) {
            worker_thread_.join();
        }
#ifdef MUSE_SMC_DEBUG
        std::cerr << "Ended filter core! \n";
#endif
        return true;
    }

    inline void addPrediction(const typename prediction_t::Ptr &prediction)
    {
        prediction_queue_.emplace(prediction);
        notify_prediction_.notify_one();
#ifdef MUSE_SMC_LOG_STATE
        log();
#endif
#ifdef MUSE_SMC_DEBUG
        std::cerr << "Added prediction! \n";
#endif
    }

    inline void addUpdate(const typename update_t::Ptr &update)
    {
        if (enable_lag_correction_) {
            const std::size_t        id    = update->getModelId();
            const cslibs_time::Time &stamp = update->getStamp();

            cslibs_time::statistics::DurationLowpass &lag = lag_map_[id];
            lag += cslibs_time::Duration(
                        static_cast<int64_t>(std::max(0L, update->getStampReceived().nanoseconds() - update->getStamp().nanoseconds())));
            if (lag.duration() >= lag_) {
                lag_ = lag.duration();
                lag_source_ = id;
#ifdef MUSE_SMC_DEBUG
                std::cerr << "lag " << id << " " << lag.duration() << " " << (update->getStampReceived() - update->getStamp()) << "\n";
#endif
#ifdef MUSE_SMC_DEBUG
            std::cerr << "Added update! \n";
#endif
            }
            if (id == lag_source_) {
                while (delayed_update_queue_.hasElements()) {
                    if (delayed_update_queue_.top()->getStamp() <= stamp)
                        update_queue_.emplace(delayed_update_queue_.pop());
                    else
                        break;

                }
                update_queue_.emplace(update);
                notify_event_.notify_one();
            } else {
                delayed_update_queue_.emplace(update);
            }
#ifdef MUSE_SMC_LOG_STATE
            log();
#endif
        } else {
            update_queue_.emplace(update);
            notify_event_.notify_one();
        }
    }

    inline void requestStateInitialization(const state_t &state,
                                           const covariance_t &covariance)
    {
        lock_t l(init_state_mutex_);
        init_state_             = state;
        init_state_covariance_  = covariance;
        request_init_state_     = true;
#ifdef MUSE_SMC_DEBUG
        std::cerr << "State initialization requested! \n";
#endif
    }

    void requestUniformInitialization()
    {
        request_init_uniform_   = true;
#ifdef MUSE_SMC_DEBUG
        std::cerr << "Uniform initialization requested! \n";
#endif
    }

protected:
    /// functions to apply to the sample set
    typename sample_set_t::Ptr              sample_set_;
    typename uniform_sampling_t::Ptr        sample_uniform_;
    typename normal_sampling_t::Ptr         sample_normal_;
    typename resampling_t::Ptr              resampling_;
    typename prediction_integrals_t::Ptr    prediction_integrals_;
    typename scheduler_t::Ptr               scheduler_;
    typename filter_state_t::Ptr            state_publisher_;

    /// requests
    std::mutex                              init_state_mutex_;
    state_t                                 init_state_;
    covariance_t                            init_state_covariance_;
    atomic_bool_t                           request_init_state_;
    atomic_bool_t                           request_init_uniform_;
    atomic_bool_t                           request_update_uniform_;

    /// processing queues
    update_queue_t                          update_queue_;
    update_queue_t                          delayed_update_queue_;
    prediction_queue_t                      prediction_queue_;
    duration_map_t                          lag_map_;
    duration_t                              lag_;
    std::size_t                             lag_source_;
    bool                                    enable_lag_correction_;

    /// background thread
    mutex_t                                 worker_thread_mutex_;
    thread_t                                worker_thread_;
    atomic_bool_t                           worker_thread_active_;
    atomic_bool_t                           worker_thread_exit_;
    condition_variable_t                    notify_event_;
    mutable mutex_t                         notify_event_mutex_;
    condition_variable_t                    notify_prediction_;
    mutable mutex_t                         notify_prediction_mutex_;

#ifdef MUSE_SMC_USE_DOTTY
    Dotty::Ptr                          dotty_;
#endif
#ifdef MUSE_SMC_LOG_STATE
    SMCFilterStateLogger::Ptr           log_;
    void log()
    {
        log_->log(prediction_queue_.size(),
                  update_queue_.size(),
                  sample_set_->getStamp().seconds() / ros::Time::now().toSec());
    }
#endif

    inline bool hasUpdates()
    {
        return !update_queue_.empty();
    }

    inline bool hasPredictions()
    {
        return !prediction_queue_.empty();
    }

    inline void requests()
    {
        if (request_update_uniform_)
            if (sample_uniform_->update(sample_set_->getFrame()))
                request_update_uniform_ = false;

        if (request_init_uniform_) {
            if (sample_uniform_->apply(*sample_set_)) {
                state_publisher_->publishIntermediate(sample_set_);
                request_init_uniform_   = false;
                request_update_uniform_ = false;
            }
        }

        if (request_init_state_) {
            if (sample_normal_->apply(init_state_,
                                      init_state_covariance_,
                                      *sample_set_)) {
                state_publisher_->publish(sample_set_);
                request_init_state_  = false;
            }
        }
    }

    inline void predict(const cslibs_time::Time &until)
    {
        auto wait_for_prediction = [this] () {
            lock_t l(notify_prediction_mutex_);
            notify_prediction_.wait(l);
        };

        const cslibs_time::Time &time_stamp = sample_set_->getStamp();
        while (until > time_stamp) {
            if (prediction_queue_.empty())
                wait_for_prediction();

            typename prediction_t::Ptr prediction = prediction_queue_.pop();
            if (prediction->getStamp() < time_stamp) {
                /// drop odometry messages which are too old
                continue;
            }

            /// mutate time stamp
            typename prediction_result_t::Ptr prediction_result = prediction->apply(until, sample_set_->getStateIterator());
            if (prediction_result->success()) {
                prediction_integrals_->add(prediction_result);
                sample_set_->setStamp(prediction_result->applied->getTimeFrame().end);

#ifdef MUSE_SMC_USE_DOTTY
                dotty_->addPrediction(prediction_result->applied->getTimeFrame().end, static_cast<bool>(prediction_result->left_to_apply));
#endif

                if (prediction_result->left_to_apply) {
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

    inline void loop()
    {
        worker_thread_active_ = true;
        lock_t notify_event_mutex_lock(notify_event_mutex_);

        request_update_uniform_  = true;

#ifdef MUSE_SMC_DEBUG
        cslibs_time::Time     last = cslibs_time::Time::now();
        cslibs_time::Time     now;
        cslibs_time::Duration dur;
        cslibs_math::statistics::Mean<1> mean_rate;
#endif
        while (!worker_thread_exit_) {
            requests();

            notify_event_.wait(notify_event_mutex_lock);
            if (worker_thread_exit_)
                break;

            while (update_queue_.hasElements()) {
                if (worker_thread_exit_)
                    break;

                requests();

                typename update_t::Ptr   u = update_queue_.pop();
                const cslibs_time::Time &t = u->getStamp();
                const cslibs_time::Time &sample_set_stamp = sample_set_->getStamp();

                if (t >= sample_set_stamp) {

                    predict(t);

                    if (t > sample_set_stamp) {
                        update_queue_.emplace(u);
                    } else if (t == sample_set_stamp) {
                        const auto model_id = u->getModelId();
                        if (!prediction_integrals_->isZero(model_id)) {
                            if (scheduler_->apply(u, sample_set_)) {
                                resampling_->updateRecovery(*sample_set_);
                                prediction_integrals_->reset(model_id);
                            }
                            state_publisher_->publishIntermediate(sample_set_);
#ifdef MUSE_SMC_LOG_STATE
                            log();
#endif
#ifdef MUSE_SMC_USE_DOTTY
                            dotty_->addState(sample_set_stamp);
                            dotty_->addUpdate(u->getStamp(), u->getModelName());
#endif
                        }
                    }
                }
#ifdef MUSE_SMC_DEBUG
                else {
                    std::cerr << "Dropped " << u->getModelName() << " " << (sample_set_->getStamp() - u->getStamp()).milliseconds() << std::endl;
                }
#endif
                if (prediction_integrals_->thresholdExceeded() &&
                        scheduler_->apply(resampling_, sample_set_)) {
                    prediction_integrals_->reset();
                    state_publisher_->publish(sample_set_);
                }
#ifdef MUSE_SMC_DEBUG
                now = cslibs_time::Time::now();
                dur = now - last;
                if(!dur.isZero() && !prediction_integrals_->isZero()) {
                    const double rate = 1.0 / dur.seconds();
                    mean_rate.add(rate);
                    std::cout << "[MuseSMC]: \n"
                              << " Mean rate   : " << mean_rate.get() << "Hz \n"
                              << " current rate: " << rate << "Hz \n"
                              << " sample size : " << sample_set_->getSampleSize() << "\n"
                              << " update queue: " << update_queue_.size() << "\n"
                              << " pred. queue : " << prediction_queue_.size() << "\n";
                }
                last = now;
#endif
            }
        }
        worker_thread_active_ = false;
    }
};
}

#endif // FILTER_HPP
