#ifndef SAMPLE_SET_HPP
#define SAMPLE_SET_HPP

#include <string>
#include <limits>

#include <muse_mcl/utility/buffered_vector.hpp>
#include <muse_mcl/utility/member_iterator.hpp>

#include <muse_mcl/particle_filter_v2/sample.hpp>
#include <muse_mcl/particle_filter_v2/sample_insertion.hpp>
#include <muse_mcl/particle_filter_v2/sample_indexation.hpp>

namespace muse_mcl {
template<typename StateT, typename Dimension>
class SampleSet
{
public:
    using Ptr               = std::shared_ptr<sample_set_t>;
    using ConstPtr          = std::shared_ptr<sample_set_t const>;
    using weight_iterator_t = MemberDecorator<Sample<StateT>, double, &Sample<StateT>::weight>;
    using state_iterator_t  = MemberDecorator<Sample<StateT>, StateT, &Sample<StateT>::state>;
    using indexation_t      = SampleIndexation<StateT, Dimension>;
    using index_vector_t    = std::buffered_vector<indexation_t::index_t>;
    using sample_vector_t   = std::buffered_vector<Sample<StateT>>;
    using sample_set_t      = SampleSet<StateT, Dimension>;

    /// Allowed and deleted constructors
    SampleSet(const SampleSet &other) = delete;
    SampleSet& operator = (const SampleSet &other) = delete;

    /**
     * @brief SampleSet constructor.
     * @param frame_id      - the coordinate frame, the samples are defined in
     * @param sample_size   - the sample_size of the set, which is fixed using
     *                        this constructor
     */
    SampleSet(const std::string &frame_id,
              const std::size_t sample_size,
              const SampleIndexation<StateT>::Ptr &indexation) :
        frame_id_(frame_id),
        minimum_sample_size_(sample_size),
        maximum_sample_size_(sample_size),
        maximum_weight_(1.0 / sample_size_minimum),
        average_weight_(1.0 / sample_size_minimum),
        weight_sum_(1.0),
        p_t_1_(new sample_vector_t(0, sample_size)),
        p_t_1_indeces_(new index_vector_t(0, sample_size)),
        p_t_(new sample_vector_t(0, sample_size)),
        indexation_(indexation)
    {
    }

    /**
     * @brief SampleSet constructor.
     * @param frame_id              - the coordinate frame, the samples are defined in
     * @param sample_size_minimum   - the minimum sample size allowed for the set
     * @param sample_size_maxmimum  - the maximum sample size allowed for the set
     */
    SampleSet(const std::string &frame_id,
              const std::size_t sample_size_minimum,
              const std::size_t sample_size_maxmimum,
              const SampleIndexation<StateT>::Ptr &indexation) :
        frame_id_(frame_id),
        minimum_sample_size_(sample_size_minimum),
        maximum_sample_size_(sample_size_maxmimum),
        maximum_weight_(1.0 / sample_size_minimum),
        average_weight_(1.0 / sample_size_minimum),
        weight_sum_(1.0),
        p_t_1_(new sample_vector_t(0, sample_size_maximum)),
        p_t_1_indeces_(new index_vector_t(0, sample_size_maximum)),
        p_t_(new sample_vector_t(0, sample_size_maximum)),
        indexation_(indexation)
    {
    }

    /**
     * @brief   Move assignment operator
     * @param   other - the particle set to be moved
     */
    inline SampleSet& operator = (SampleSet &&other) = default;


    inline weight_iterator_t getWeightIterator()
    {
        return weight_iterator_t(*p_t_1_,
                              weight_iterator_t::notify_update::from<sample_set_t, &sample_set_t::weightUpdate>(this),
                              weight_iterator_t::notify_touch::from<sample_set_t, &sample_set_t::resetSampleWeightStatistic>(this));
    }

    inline state_iterator_t getStateIterator()
    {
        return state_iterator_t(*p_t_1_,
                              state_iterator_t::notify_update::from<sample_set_t, &sample_set_t::indexationUpdate>(this),
                              state_iterator_t::notify_touch::from<sample_set_t, &sample_set_t::resetIndexationStatistic>(this));
    }

    inline SampleInsertion<StateT> getInsertion()
    {
        resetSampleWeightStatistic();
        resetIndexationStatistic();
        p_t_->clear();
        return SampleInsertion<StateT>(*p_t_,
                                       SampleInsertion::notify_update::from<sample_set_t, &SampleSet::insertionUpdate>(this),
                                       SampleInsertion::notify_closed::from<sample_set_t, &SampleSet::insertionClosed>(this));
    }


    /**
     * @brief normalizeWeights the sample weights.
     */
    inline void normalizeWeights()
    {
        if(p_t_1_->size() == 0 || weight_sum_ == 0.0) {
            return;
        }
        for(auto &s : *p_t_1_) {
            s.weight /= weight_sum_;
        }
        average_weight_ /= weight_sum_;
        maximum_weight_ /= weight_sum_;
        weight_sum_ = 1.0;
    }

    /**
     * @brief resetWeights resets the weights ever to a normalized uniform distribution or a non-normalized one.
     * @param set_to_one    - sets all sample weights to one
     */
    inline void resetWeights(const bool set_to_one = false)
    {
        if(p_t_1_->size() == 0) {
            return;
        }
        const double weight = set_to_one ? 1.0 : 1.0 / static_cast<double>(p_t_1_->size());
        for(auto &s : *p_t_1_) {
            s.weight = weight;
        }
        average_weight_ = weight;
        maximum_weight_ = weight;
        weight_sum_     = weight * p_t_1_->size();
    }


    /**
     * @brief getMinimumSampleSize returns the minimum  of allowed sample sizes.
     * @return  - minimum_sample_size_
     */
    inline std::size_t getMinimumSampleSize() const
    {
        return minimum_sample_size_;
    }

    /**
     * @brief getMaximumSampleSize return the maximum of allowed sample sizes.
     * @return  - maximum_sample_size_
     */
    inline std::size_t getMaximumSampleSize() const
    {
        return maximum_sample_size_;
    }

    /**
     * @brief getSampleSize returns the current sample size of the set.
     * @return  - size of current set p_t_1_
     */
    inline std::size_t getSampleSize() const
    {
         return p_t_1_->size();
    }

    /**
     * @brief getFrameID returns the frame id.
     * @return      - frame_id_
     */
    inline std::string const & getFrameID() const
    {
        return frame_id_;
    }

    /**
     * @brief getMaximumWeight returns the maximum sample weight.
     * @return      - maximum_weight_
     */
    inline double getMaximumWeight() const
    {
        return maximum_weight_;
    }

    /**
     * @brief getAverageWeight returns the average weight.
     * @return      - average_weight_
     */
    inline double getAverageWeight() const
    {
        return average_weight_;
    }

    /**
     * @brief getWeightSum returns the weight sum.
     * @return      - weight_sum_
     */
    inline double getWeightSum() const
    {
        return weight_sum_;
    }

    /**
     * @brief isNormalized returns if the sample weights are normalized.
     * @return      - weight_sum == 1.0
     */
    inline bool isNormalized() const
    {
        return weight_sum_ == 1.0;
    }

    /**
     * @brief getSamples returns the sample vector / set contents.
     * @return  - *p_t_1_
     */
    inline sample_vector_t const & getSamples() const
    {
        return *p_t_1_;
    }

    /**
     * @brief getIndices returns the sample index vector / set contents.
     * @return  - *p_t_1_indices
     */
    inline index_vector_t const & getIndices() const
    {
        return *p_t_1_indeces_;
    }

private:
    std::string             frame_id_;
    std::size_t             minimum_sample_size_;
    std::size_t             maximum_sample_size_;

    double                  maximum_weight_;
    double                  average_weight_;
    double                  weight_sum_;

    sample_vector_t::Ptr    p_t_1_;
    index_vector_t::Ptr     p_t_1_indeces_;
    sample_vector_t::Ptr    p_t_;

    indexation_t::Ptr       indexation_;
    indexation_t::index_t   minimum_index_;
    indexation_t::index_t   maximum_index_;

    void resetSampleWeightStatistic()
    {
        maximum_weight_ = 0.0;
        average_weight_ = 0.0;
        weight_sum_     = 0.0;
    }

    void resetIndexationStatistic()
    {
        minimum_index_  = std::numeric_limits<int>::max();
        maximum_index_  = std::numeric_limits<int>::min();
        indeces_.clear();
    }

    void indexationUpdate(const StateT &state)
    {
        const indexation_t::index_t i = indexation_->create(state);
        minimum_index_.min(i);
        maximum_index_.max(i);
        p_t_1_indeces_->emplace_back(i);
    }

    void weightUpdate(const double weight)
    {
        weight_sum_ += weight;
        if(weight > maximum_weight_)
            maximum_weight_ = weight;
        average_weight_ = weight_sum_ / p_t_1_->size();
    }

    void insertionUpdate(const Sample<StateT> &sample)
    {
        stateUpdate(sample.state);
        weightUpdate(sample.weight);
    }

    void insertionClosed()
    {
        std::swap(p_t_, p_t_1_);
    }
};
}


#endif // SAMPLE_SET_HPP
