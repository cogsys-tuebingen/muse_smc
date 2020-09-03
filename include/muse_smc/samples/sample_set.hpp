#ifndef MUSE_SMC_SAMPLE_SET_HPP
#define MUSE_SMC_SAMPLE_SET_HPP

#include <cslibs_math/statistics/distribution.hpp>
#include <cslibs_utility/buffered/buffered_vector.hpp>
#include <limits>
#include <muse_smc/samples/sample_density.hpp>
#include <muse_smc/samples/sample_insertion.hpp>
#include <muse_smc/samples/sample_state_iterator.hpp>
#include <muse_smc/samples/sample_weight_iterator.hpp>
#include <muse_smc/smc/traits/sample.hpp>
#include <string>

namespace muse_smc {
template <typename Sample_T, typename State_T, typename Weight_T, typename Time_T>
class EIGEN_ALIGN16 SampleSet {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using sample_t = Sample_T;
  using type = SampleSet<Sample_T, State_T, Weight_T, Time_T>;
  using sample_vector_t =
      cslibs_utility::buffered::buffered_vector<Sample_T,
                                                typename Sample_T::allocator_t>;
  using sample_density_t = SampleDensity<Sample_T>;
  using sample_insertion_t = SampleInsertion<Sample_T>;
  using state_iterator_t = StateIteration<Sample_T, State_T, Weight_T, Time_T>;
  using weight_iterator_t = WeightIteration<Sample_T, State_T, Weight_T>;
  using weight_distribution_t =
      cslibs_math::statistics::Distribution<Weight_T, 1>;

  /**
   * @brief SampleSet deleted - non-copyable
   * @param other
   */
  SampleSet(const SampleSet &other) = delete;
  /**
   * @brief operator = deleted - non-copyable
   * @param other
   * @return
   */
  SampleSet &operator=(const SampleSet &other) = delete;

  /**
   * @brief SampleSet constructor.
   * @param frame_id              frame id samples are defined in
   * @param time_stamp            time stamp at which samples are valid
   * @param sample_size           fixed sample size -> min = max = samplesize
   * @param density               the density estimation function
   * @param reset_weights_to_one  after insertion weight is either set to 1 / N
   * or 1
   */
  inline explicit SampleSet(const std::string &frame_id,
                            const Time_T &time_stamp,
                            const std::size_t sample_size,
                            const std::shared_ptr<sample_density_t> &density,
                            const bool keep_weights_after_insertion = false)
      : frame_id_{frame_id},
        stamp_{time_stamp},
        minimum_sample_size_{sample_size},
        maximum_sample_size_{sample_size},
        p_t_1_{new sample_vector_t(0, maximum_sample_size_)},
        p_t_1_density_{density},
        p_t_{new sample_vector_t(0, maximum_sample_size_)},
        keep_weights_after_insertion_{keep_weights_after_insertion} {}

  /**
   * @brief SampleSet constructor.
   * @param frame_id              frame id samples are defined in
   * @param time_stamp            time stamp at which samples are valid
   * @param sample_size_minimum   minimum sample size of the set
   * @param sample_size_maximum  maximum sample size of the set
   * @param density               the density estimation function
   * @param reset_weights_to_one  after insertion weight is either set to 1 / N
   * or 1
   */
  inline explicit SampleSet(const std::string &frame_id,
                            const Time_T &time_stamp,
                            const std::size_t sample_size_minimum,
                            const std::size_t sample_size_maximum,
                            const std::shared_ptr<sample_density_t> &density,
                            const bool keep_weights_after_insertion = false)
      : frame_id_{frame_id},
        stamp_{time_stamp},
        minimum_sample_size_{sample_size_minimum},
        maximum_sample_size_{sample_size_maximum},
        p_t_1_{new sample_vector_t(0, maximum_sample_size_)},
        p_t_1_density_{density},
        p_t_{new sample_vector_t(0, maximum_sample_size_)},
        keep_weights_after_insertion_{keep_weights_after_insertion} {}

  /**
   * @brief Destructor.
   */
  virtual ~SampleSet() = default;

  /**
   * @brief Move operator.
   * @param other                 set to be moved
   */
  inline SampleSet &operator=(SampleSet &&other) = default;

  inline weight_iterator_t getWeightIterator() {
    return weight_iterator_t(*p_t_1_,
                             weight_iterator_t::notify_touch::template from<
                                 type, &type::weightStatisticReset>(this),
                             weight_iterator_t::notify_update::template from<
                                 type, &type::weightUpdate>(this),
                             weight_iterator_t::notify_finished::template from<
                                 type, &type::normalizeWeights>(this));
  }

  inline state_iterator_t getStateIterator() {
    return state_iterator_t(stamp_, *p_t_1_);
  }

  inline sample_insertion_t getInsertion() {
    weightStatisticReset();
    p_t_1_density_->clear();
    p_t_->clear();
    return sample_insertion_t(*p_t_,
                              sample_insertion_t::notify_update::template from<
                                  type, &type::insertionUpdate>(this),
                              sample_insertion_t::notify_closed::template from<
                                  type, &type::insertionClosedReset>(this));
  }

  inline void normalizeWeights() {
    if (p_t_1_->size() == 0) {
      return;
    }
    if (weight_sum_ == 0.0) {
      resetWeights();
    }

    weight_distribution_.reset();
    for (auto &s : *p_t_1_) {
      s.weight() /= weight_sum_;
      weight_distribution_.add(s.weight());
    }
    maximum_weight_ /= weight_sum_;
    weight_sum_ = 1.0;
  }

  inline void resetWeights() {
    if (p_t_1_->size() == 0) return;

    weight_distribution_.reset();
    for (auto &s : *p_t_1_) {
      s.weight() = 1.0;
      weight_distribution_.add(1.0);
    }

    maximum_weight_ = 1.0;
    weight_sum_ = static_cast<Weight_T>(p_t_1_->size());
  }

  inline std::size_t getMinimumSampleSize() const {
    return minimum_sample_size_;
  }

  inline std::size_t getMaximumSampleSize() const {
    return maximum_sample_size_;
  }

  inline std::size_t getSampleSize() const { return p_t_1_->size(); }

  inline std::string const &getFrame() const { return frame_id_; }

  inline Time_T const &getStamp() const { return stamp_; }

  inline void setStamp(const Time_T &time) { stamp_ = time; }

  inline Weight_T getMinimumWeight() const { return minimum_weight_; }

  inline Weight_T getMaximumWeight() const { return maximum_weight_; }

  inline Weight_T getAverageWeight() const {
    return weight_distribution_.getMean();
  }

  inline weight_distribution_t const &getWeightDistribution() const {
    return weight_distribution_;
  }

  inline Weight_T getWeightSum() const { return weight_sum_; }

  inline Weight_T getWeightVariance() const {
    return weight_distribution_.getVariance();
  }

  inline bool isNormalized() const { return weight_sum_ == 1.0; }

  inline sample_vector_t const &getSamples() const { return *p_t_1_; }

  inline std::shared_ptr<sample_density_t const> getDensity() const {
    return p_t_1_density_;
  }

  inline void updateDensity() const {
    p_t_1_density_->clear();
    for (const auto &s : *p_t_1_) p_t_1_density_->insert(s);
    p_t_1_density_->estimate();
  }

 private:
  std::string frame_id_;
  Time_T stamp_;
  std::size_t minimum_sample_size_{0};
  std::size_t maximum_sample_size_{0};

  Weight_T maximum_weight_{0.0};
  Weight_T minimum_weight_{std::numeric_limits<Weight_T>::max()};
  weight_distribution_t weight_distribution_;
  Weight_T weight_sum_{0.0};

  std::shared_ptr<sample_vector_t> p_t_1_{nullptr};
  mutable std::shared_ptr<sample_density_t> p_t_1_density_{nullptr};
  std::shared_ptr<sample_vector_t> p_t_{nullptr};

  bool keep_weights_after_insertion_;

  inline void weightStatisticReset() {
    maximum_weight_ = 0.0;
    minimum_weight_ =
        std::numeric_limits<Weight_T>::max();  /// for update functions that yield
                                             /// values higher than 1.0
    weight_distribution_.reset();
    weight_sum_ = 0.0;
  }

  inline void weightUpdate(const Weight_T weight) {
    weight_sum_ += weight;
    maximum_weight_ = weight > maximum_weight_ ? weight : maximum_weight_;
    minimum_weight_ = weight < minimum_weight_ ? weight : minimum_weight_;
  }

  inline void insertionUpdate(const sample_t &sample) {
    weightUpdate(sample.weight());
    p_t_1_density_->insert(sample);
  }

  inline void insertionClosedReset() {
    std::swap(p_t_, p_t_1_);
    p_t_1_density_->estimate();
    if (keep_weights_after_insertion_) normalizeWeights();
  }
};
}  // namespace muse_smc

#endif  // MUSE_SMC_SAMPLE_SET_HPP
