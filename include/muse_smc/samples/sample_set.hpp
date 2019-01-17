#ifndef SAMPLE_SET_HPP
#define SAMPLE_SET_HPP

#include <string>
#include <limits>

#include <cslibs_utility/buffered/buffered_vector.hpp>

#include <cslibs_time/time.hpp>

#include <cslibs_math/statistics/distribution.hpp>

#include <muse_smc/samples/sample_density.hpp>
#include <muse_smc/samples/sample_insertion.hpp>
#include <muse_smc/samples/sample_weight_iterator.hpp>
#include <muse_smc/samples/sample_state_iterator.hpp>

namespace muse_smc {
template<typename state_space_description_t>
class EIGEN_ALIGN16 SampleSet
{
public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using sample_t              = typename state_space_description_t::sample_t;
    using sample_set_t          = SampleSet<state_space_description_t>;
    using sample_vector_t       = cslibs_utility::buffered::buffered_vector<sample_t, typename sample_t::allocator_t>;
    using sample_density_t      = SampleDensity<sample_t>;
    using sample_insertion_t    = SampleInsertion<sample_t>;
    using state_iterator_t      = StateIteration<state_space_description_t>;
    using weight_iterator_t     = WeightIteration<state_space_description_t>;
    using weight_distribution_t = cslibs_math::statistics::Distribution<1>;

    using Ptr = std::shared_ptr<sample_set_t>;
    using ConstPtr = std::shared_ptr<sample_set_t const>;

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
    SampleSet& operator = (const SampleSet &other) = delete;

    /**
     * @brief SampleSet constructor.
     * @param frame_id              frame id samples are defined in
     * @param time_stamp            time stamp at which samples are valid
     * @param sample_size           fixed sample size -> min = max = samplesize
     * @param density               the density estimation function
     * @param reset_weights_to_one  after insertion weight is either set to 1 / N or 1
     */
    SampleSet(const std::string                    &frame_id,
              const cslibs_time::Time              &time_stamp,
              const std::size_t                     sample_size,
              const typename sample_density_t::Ptr &density) :
        frame_id_(frame_id),
        stamp_(time_stamp),
        minimum_sample_size_(sample_size),
        maximum_sample_size_(sample_size),
        maximum_weight_(0.0),
        weight_sum_(0.0),
        p_t_1_(new sample_vector_t(0, maximum_sample_size_)),
        p_t_1_density_(density),
        p_t_(new sample_vector_t(0, maximum_sample_size_))
    {
    }

    /**
     * @brief SampleSet constructor.
     * @param frame_id              frame id samples are defined in
     * @param time_stamp            time stamp at which samples are valid
     * @param sample_size_minimum   minimum sample size of the set
     * @param sample_size_maxmimum  maximum sample size of the set
     * @param density               the density estimation function
     * @param reset_weights_to_one  after insertion weight is either set to 1 / N or 1
     */
    SampleSet(const std::string                    &frame_id,
              const cslibs_time::Time              &time_stamp,
              const std::size_t                     sample_size_minimum,
              const std::size_t                     sample_size_maxmimum,
              const typename sample_density_t::Ptr &density) :
        frame_id_(frame_id),
        stamp_(time_stamp),
        minimum_sample_size_(sample_size_minimum),
        maximum_sample_size_(sample_size_maxmimum),
        maximum_weight_(0.0),
        weight_sum_(0.0),
        p_t_1_(new sample_vector_t(0, maximum_sample_size_)),
        p_t_1_density_(density),
        p_t_(new sample_vector_t(0, maximum_sample_size_))
    {
    }

    /**
     * @brief Destructor.
     */
    virtual ~SampleSet() = default;

    /**
     * @brief Move operator.
     * @param other                 set to be moved
     */
    inline SampleSet& operator = (SampleSet &&other) = default;

    inline weight_iterator_t getWeightIterator()
    {
        return weight_iterator_t(*p_t_1_,
                                weight_iterator_t::notify_touch::template    from<sample_set_t, &sample_set_t::weightStatisticReset>(this),
                                weight_iterator_t::notify_update::template   from<sample_set_t, &sample_set_t::weightUpdate>(this),
                                weight_iterator_t::notify_finished::template from<sample_set_t, &sample_set_t::normalizeWeights>(this));
    }

    inline state_iterator_t getStateIterator()
    {
        return state_iterator_t(stamp_, *p_t_1_);
    }

    inline sample_insertion_t getInsertion()
    {
        weightStatisticReset();
        p_t_1_density_->clear();
        p_t_->clear();
        return sample_insertion_t(*p_t_,
                                  sample_insertion_t::notify_update::template from<sample_set_t, &sample_set_t::insertionUpdate>(this),
                                  sample_insertion_t::notify_closed::template from<sample_set_t, &sample_set_t::insertionClosedReset>(this));
    }

    inline void normalizeWeights()
    {
        if (p_t_1_->size() == 0)
            return;
        if (weight_sum_ == 0.0)
            resetWeights();

        weight_distribution_.reset();
        for (auto &s : *p_t_1_) {
            s.weight /= weight_sum_;
            weight_distribution_.add(s.weight);
        }
        maximum_weight_ /= weight_sum_;
        weight_sum_ = 1.0;
    }

    inline void resetWeights()
    {
        if (p_t_1_->size() == 0)
            return;

        weight_distribution_.reset();
        for (auto &s : *p_t_1_) {
            s.weight = 1.0;
            weight_distribution_.add(1.0);
        }

        maximum_weight_ = 1.0;
        weight_sum_     = static_cast<double>(p_t_1_->size());
    }

    inline std::size_t getMinimumSampleSize() const
    {
        return minimum_sample_size_;
    }

    inline std::size_t getMaximumSampleSize() const
    {
        return maximum_sample_size_;
    }

    inline std::size_t getSampleSize() const
    {
         return p_t_1_->size();
    }

    inline std::string const & getFrame() const
    {
        return frame_id_;
    }

    inline cslibs_time::Time const & getStamp() const
    {
        return stamp_;
    }

    inline void setStamp(const cslibs_time::Time &time)
    {
        stamp_ = time;
    }

    inline double getMinimumWeight() const
    {
        return minimum_weight_;
    }

    inline double getMaximumWeight() const
    {
        return maximum_weight_;
    }

    inline double getAverageWeight() const
    {
        return weight_distribution_.getMean();
    }

    inline weight_distribution_t const & getWeightDistribution() const
    {
        return weight_distribution_;
    }

    inline double getWeightSum() const
    {
        return weight_sum_;
    }

    inline double getWeightVariance() const
    {
        return weight_distribution_.getVariance();
    }

    inline bool isNormalized() const
    {
        return weight_sum_ == 1.0;
    }

    inline sample_vector_t const & getSamples() const
    {
        return *p_t_1_;
    }

    inline typename sample_density_t::ConstPtr getDensity() const
    {
        return p_t_1_density_;
    }

    inline void updateDensity() const
    {
        p_t_1_density_->clear();
        for (const auto &s : *p_t_1_)
            p_t_1_density_->insert(s);
        p_t_1_density_->estimate();
    }

private:
    std::string                                 frame_id_;
    cslibs_time::Time                           stamp_;
    std::size_t                                 minimum_sample_size_;
    std::size_t                                 maximum_sample_size_;

    double                                      maximum_weight_;
    double                                      minimum_weight_;
    weight_distribution_t                       weight_distribution_;
    double                                      weight_sum_;

    std::shared_ptr<sample_vector_t>            p_t_1_;
    mutable typename sample_density_t::Ptr      p_t_1_density_;
    std::shared_ptr<sample_vector_t>            p_t_;

    inline void weightStatisticReset()
    {
        maximum_weight_ = 0.0;
        minimum_weight_ = std::numeric_limits<double>::max(); /// for update functions that yield values higher than 1.0
        weight_distribution_.reset();
        weight_sum_     = 0.0;
    }

    inline void weightUpdate(const double weight)
    {
        weight_sum_    += weight;
        maximum_weight_ = weight > maximum_weight_ ? weight : maximum_weight_;
        minimum_weight_ = weight < minimum_weight_ ? weight : minimum_weight_;
    }

    inline void insertionUpdate(const sample_t &sample)
    {
        weightUpdate(sample.weight);
        p_t_1_density_->insert(sample);
    }

    inline void insertionClosedReset()
    {
        std::swap(p_t_, p_t_1_);
        p_t_1_density_->estimate();
    }
};
}

#endif // SAMPLE_SET_HPP
