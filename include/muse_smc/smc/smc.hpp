#ifndef MUSE_SMC_HPP
#define MUSE_SMC_HPP

/// PROJECT
#include <muse_smc/smc/traits/prediction.hpp>
#include <muse_smc/smc/traits/prediction_integrals.hpp>
#include <muse_smc/smc/traits/requests.hpp>
#include <muse_smc/smc/traits/resampling.hpp>
#include <muse_smc/smc/traits/scheduler.hpp>
#include <muse_smc/smc/traits/state_publisher.hpp>
#include <muse_smc/smc/traits/update.hpp>

/// CSLIBS
#include <cslibs_time/statistics/duration_lowpass.hpp>
#include <cslibs_utility/synchronized/synchronized_priority_queue.hpp>

/// SYSTEM
#include <atomic>
#include <condition_variable>
#include <memory>
#include <queue>
#include <thread>
#include <unordered_map>

/***
 * Distance thresholds for resampling and update throttling are
 * currently left out of the game. Questionable is, if time
 * thresholds respectively rates can be used instead.
 */

namespace muse_smc {
template <typename Sample_T>
class SMC {
 public:
  /// utility typedefs
  using Ptr = std::shared_ptr<SMC<Sample_T>>;
  using update_queue_t = cslibs_utility::synchronized::priority_queue<
      typename traits::Update<Sample_T>::type::Ptr,
      typename traits::Update<Sample_T>::type::Greater>;
  using prediction_queue_t = cslibs_utility::synchronized::priority_queue<
      typename traits::Prediction<Sample_T>::type::Ptr,
      typename traits::Prediction<Sample_T>::type::Greater>;
  using duration_map_t =
      std::unordered_map<std::size_t, cslibs_time::statistics::DurationLowpass>;

  /**
   * @brief SMC default constructor.
   */
  inline SMC() = default;
  /**
   * @brief ~SMC default destructor.
   */
  virtual ~SMC() { end(); }

  /**
   * @brief Setup the filter with every function that is required for sampling,
   * scheduling and so on.
   * @param sample_set                                - the sample set to apply
   * the filter to
   * @param sample_uniform                            - the uniform sampler
   * @param sample_normal                             - the normally distributed
   * sampler
   * @param resampling                                - the resampling scheme to
   * be used
   * @param state_publisher                           - the state publisher,
   * which communicates the filter state to the outside world
   * @param prediction_integrals                      - the prediction
   * integrals, which accumulate if the samples were moved
   * @param scheduler                                 - the scheduling scheme
   * @param reset_all_model_accumulators_on_update    - reset all model
   * accumulators when an update is carried out
   * @param reset_model_accumulators_after_resampling - reset all model
   * accumlators after resampling is carried out
   * @param enable_lag_correction                     - lag correction for
   * delayed update inputs
   */
  inline void setup(const sample_set_t::Ptr &sample_set,
                    const uniform_sampling_t::Ptr &sample_uniform,
                    const normal_sampling_t::type::Ptr &sample_normal,
                    const resampling_t::Ptr &resampling,
                    const state_publisher_t::Ptr &state_publisher,
                    const prediction_integrals_t::Ptr &prediction_integrals,
                    const scheduler_t::Ptr &scheduler,
                    const bool reset_all_model_accumulators_on_update,
                    const bool reset_model_accumulators_after_resampling,
                    const bool enable_lag_correction) {
    sample_set_ = sample_set;
    sample_uniform_ = sample_uniform;
    sample_normal_ = sample_normal;
    resampling_ = resampling;
    state_publisher_ = state_publisher;
    prediction_integrals_ = prediction_integrals;
    scheduler_ = scheduler;
    enable_lag_correction_ = enable_lag_correction;
    reset_all_accumulators_after_update_ =
        reset_all_model_accumulators_on_update;
    reset_model_accumulators_after_resampling_ =
        reset_model_accumulators_after_resampling;
  }

  /**
   * @brief   Start the filter.
   * @return  true if start was possible, false if filter is already running
   */
  inline bool start() {
    if (!worker_thread_.joinable()) {
      std::unique_lock<std::mutex> l(worker_thread_mutex_);
      worker_thread_exit_ = false;
      worker_thread_ = std::thread([this]() { loop(); });
      return true;
    }
    return false;
  }

  /**
   * @brief Request that the background thread is stopped.
   * @return  true if ending the filter was possible, false if filter is already
   * stopped
   */
  inline bool end() {
    if (!worker_thread_.joinable()) {
      return false;
    }
    worker_thread_exit_ = true;
    notify_event_.notify_one();
    worker_thread_.join();
    return true;
  }

  /**
   * @brief Add a new prediction to the filter for sample propagation.
   * @param prediction - the prediction or control function applied to the
   * samples
   */
  inline void addPrediction(const typename prediction_t::Ptr &prediction) {
    prediction_queue_.emplace(prediction);
    notify_prediction_.notify_one();
  }

  /**
   * @brief Add an measurement update from some channel to the filter to weight
   * the samples
   * @param update    - the update function applied to the sample set
   */
  inline void addUpdate(const typename update_t::Ptr &update) {
    if (enable_lag_correction_) {
      const auto id = update->getModelId();
      const auto &stamp = update->getStamp();

      cslibs_time::statistics::DurationLowpass &lag = lag_map_[id];
      lag += typename traits::Duration<Sample_T>::type{static_cast<int64_t>(
          std::max(0L, update->stampReceived().nanoseconds() -
                           update->getStamp().nanoseconds()))};
      if (lag.duration() >= lag_) {
        lag_ = lag.duration();
        lag_source_ = id;
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
    } else {
      update_queue_.emplace(update);
      notify_event_.notify_one();
    }
  }

  void triggerEvent() { notify_event_.notify_one(); }

  /**
   * @brief Request a state based initialization, meaning that normal sampling
   * occurs around a prior estimate.
   */
  inline void requestStateInitialization(const time_t &time,
                                         const state_t &state,
                                         const covariance_t &covariance) {
    std::unique_lock<std::mutex> l(request_state_initialization_mutex_);
    request_state_initialization_.reset(
        new typename traits::RequestStateInitialization::type{time, state,
                                                              covariance});
  }

  /**
   * @brief Request a uniform initialization of the sample set.
   * @param time          - time at which the sampling should be executed
   */
  void requestUniformInitialization(const time_t &time) {
    std::unique_lock<std::mutex> l(request_uniform_initialization_mutex_);
    request_uniform_initialization_.reset(
        new typename traits::RequestUniformInitialization::type{time});
  }

 protected:
  using time_t = typename traits::Time<Sample_T>::type;
  using state_t = typename traits::State<Sample_T>::type;
  using covariance_t = typename traits::Covariance<Sample_T>::type;
  using prediction_t = typename traits::Prediction<Sample_T>::type;
  using update_t = typename traits::Update<Sample_T>::type;
  using sample_set_t = typename traits::SampleSet<Sample_T>::type;
  using uniform_sampling_t = typename traits::UniformSampling<Sample_T>::type;
  using normal_sampling_t = typename traits::NormalSampling<Sample_T>::type;
  using resampling_t = typename traits::Resampling<Sample_T>::type;
  using state_publisher_t = typename traits::StatePublisher<Sample_T>::type;
  using prediction_integrals_t =
      typename traits::PredictionIntegrals<Sample_T>::type;
  using scheduler_t = typename traits::Scheduler<Sample_T>::type;

  /// functions to apply to the sample set
  typename sample_set_t::Ptr sample_set_{nullptr};
  typename uniform_sampling_t::Ptr sample_uniform_{nullptr};
  typename normal_sampling_t::Ptr sample_normal_{nullptr};
  typename resampling_t::Ptr resampling_{nullptr};
  typename prediction_integrals_t::Ptr prediction_integrals_{nullptr};
  typename scheduler_t::Ptr scheduler_{nullptr};
  typename state_publisher_t::Ptr state_publisher_{nullptr};

  enum class Publication {
    None = 0,
    Intermediate = 1,
    Constant = 2,
    Resampling = 4
  };

  /// requests
  std::mutex request_state_initialization_mutex_;
  typename traits::RequestStateInitialization<Sample_T>::type::Ptr
      request_state_initialization_;
  std::mutex request_uniform_initialization_mutex_;
  typename traits::RequestUniformInitialization<Sample_T>::type::Ptr
      request_uniform_initialization_;

  /// processing queues
  update_queue_t update_queue_;
  update_queue_t delayed_update_queue_;
  prediction_queue_t prediction_queue_;
  duration_map_t lag_map_;
  typename traits::Duration<Sample_T>::type lag_;
  std::size_t lag_source_;
  bool enable_lag_correction_{false};
  bool has_valid_state_{false};
  bool reset_all_accumulators_after_update_{false};
  bool reset_model_accumulators_after_resampling_{false};

  /// background thread
  std::mutex worker_thread_mutex_;
  std::thread worker_thread_;
  std::atomic_bool worker_thread_exit_{false};
  std::condition_variable notify_event_;
  mutable std::mutex notify_event_mutex_;
  std::condition_variable notify_prediction_;
  mutable std::mutex notify_prediction_mutex_;

  inline void requests() {
    if (request_uniform_initialization_) {
      if (sample_uniform_->apply(*sample_set_)) {
        state_publisher_->publishIntermediate(sample_set_);
        sample_set_->setStamp(request_uniform_initialization_->time());
        request_uniform_initialization_.reset();

        has_valid_state_ = false;
        prediction_integrals_->resetAll();
        prediction_integrals_->reset();
      }
    }

    if (request_state_initialization_) {
      if (sample_normal_->apply(request_state_initialization_->state(),
                                request_state_initialization_->covariance(),
                                *sample_set_)) {
        sample_set_->setStamp(request_state_initialization_->time());
        state_publisher_->publish(sample_set_);
        request_state_initialization_.reset();

        has_valid_state_ = true;
        prediction_integrals_->resetAll();
        prediction_integrals_->reset();
      }
    }
  }

  inline void predict(const time_t &until) {
    auto wait_for_prediction = [this]() {
      std::unique_lock<std::mutex> l(notify_prediction_mutex_);
      notify_prediction_.wait(l);
    };

    const auto &time_stamp = sample_set_->getStamp();
    while (until > time_stamp) {
      if (prediction_queue_.empty()) {
        wait_for_prediction();
      }

      auto prediction = prediction_queue_.pop();
      if (prediction->getStamp() < time_stamp) {
        /// drop odometry messages which are too old
        continue;
      }

      /// mutate time stamp
      auto prediction_result =
          prediction->apply(until, sample_set_->getStateIterator());
      if (prediction_result->success()) {
        prediction_integrals_->add(prediction_result);
        sample_set_->setStamp(prediction_result->applied->timeFrame().end);

        if (prediction_result->left_to_apply) {
          typename prediction_t::Ptr prediction_left_to_apply(new prediction_t(
              prediction_result->left_to_apply, prediction->getModel()));
          prediction_queue_.emplace(prediction_left_to_apply);
          break;
        }
      } else {
        prediction_queue_.emplace(prediction);
      }
    }
  }

  inline void loop() {
    std::unique_lock<std::mutex> notify_event_mutex_lock(notify_event_mutex_);

    while (!worker_thread_exit_) {
      if (!update_queue_.hasElements())
        notify_event_.wait(notify_event_mutex_lock);

      requests();

      if (worker_thread_exit_) {
        break;
      }

      while (update_queue_.hasElements()) {
        if (worker_thread_exit_) {
          break;
        }

        requests();

        auto u = update_queue_.pop();
        const auto &t = u->getStamp();
        const auto &sample_set_stamp = sample_set_->getStamp();

        int8_t publication = has_valid_state_
                                 ? static_cast<int8_t>(Publication::Constant)
                                 : static_cast<int8_t>(Publication::None);

        if (t >= sample_set_stamp) {
          predict(t);

          if (t > sample_set_stamp) {
            update_queue_.emplace(u);
          } else if (t == sample_set_stamp) {
            const auto model_id = u->getModelId();
            if (prediction_integrals_->thresholdExceeded(model_id)) {
              if (scheduler_->apply(u, sample_set_)) {
                resampling_->updateRecovery(*sample_set_);
                if (reset_all_accumulators_after_update_)
                  prediction_integrals_->resetAll();
                else
                  prediction_integrals_->reset(model_id);
              }
              publication |= static_cast<int8_t>(Publication::Intermediate);
            }
          }
        }
        if (prediction_integrals_->thresholdExceeded() &&
            scheduler_->apply(resampling_, sample_set_)) {
          prediction_integrals_->reset();

          if (reset_model_accumulators_after_resampling_)
            prediction_integrals_->resetAll();

          publication |= static_cast<int8_t>(Publication::Resampling);
          has_valid_state_ = true;
        }

        if (publication >= static_cast<int8_t>(Publication::Resampling))
          state_publisher_->publish(sample_set_);
        else if (publication >= static_cast<int8_t>(Publication::Constant))
          state_publisher_->publishConstant(sample_set_);
        else if (publication >= static_cast<int8_t>(Publication::Intermediate))
          state_publisher_->publishIntermediate(sample_set_);
      }
    }
  }
};
}  // namespace muse_smc

#endif  // MUSE_SMC_HPP
