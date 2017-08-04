#ifndef SAMPLE_SET_HPP
#define SAMPLE_SET_HPP

#include <string>
#include <limits>

#include <muse_smc/utility/buffered_vector.hpp>
#include <muse_smc/utility/member_iterator.hpp>

#include <muse_smc/samples/sample_density.hpp>
#include <muse_smc/samples/sample_insertion.hpp>

namespace muse_smc {
template<typename sample_t>
class SampleSet
{
public:
    using sample_set_t       = SampleSet<sample_t>;
    using sample_vector_t    = std::buffered_vector<sample_t, typename sample_t::allocator_t>;
    using sample_density_t   = SampleDensity<sample_t>;
    using sample_insertion_t = SampleInsertion<sample_t>;
    using state_iterator_t   = MemberDecorator<sample_t, typename sample_t::state_t, &sample_t::state>;
    using weight_iterator_t  = MemberDecorator<sample_t, double, &sample_t::weight>;

    using Ptr = std::shared_ptr<sample_set_t>;
    using ConstPtr = std::shared_ptr<sample_set_t const>;

    SampleSet(const SampleSet &other) = delete;
    SampleSet& operator = (const SampleSet &other) = delete;

    SampleSet(const std::string           &frame_id,
              const std::size_t            sample_size,
              const sample_density_t::Ptr &density) :
        frame_id_(frame_id),
        minimum_sample_size_(sample_size),
        maximum_sample_size_(sample_size),
        maximum_weight_(1.0 / sample_size_minimum),
        average_weight_(1.0 / sample_size_minimum),
        weight_sum_(1.0),
        p_t_1_(new sample_vector_t(0, sample_size)),
        p_t_1_density_(density),
        p_t_(new sample_vector_t(0, sample_size))
    {
    }

    SampleSet(const std::string &frame_id,
              const std::size_t sample_size_minimum,
              const std::size_t sample_size_maxmimum,
              const sample_density_t::Ptr &density) :
        frame_id_(frame_id),
        minimum_sample_size_(sample_size_minimum),
        maximum_sample_size_(sample_size_maxmimum),
        maximum_weight_(1.0 / sample_size_minimum),
        average_weight_(1.0 / sample_size_minimum),
        weight_sum_(1.0),
        p_t_1_(new sample_vector_t(0, sample_size_maximum)),
        p_t_1_density_(density),
        p_t_(new sample_vector_t(0, sample_size_maximum))
    {
    }


    inline SampleSet& operator = (SampleSet &&other) = default;


    inline weight_iterator_t getWeightIterator()
    {
        return weight_iterator_t(*p_t_1_,
                              weight_iterator_t::notify_update::from<sample_set_t, &sample_set_t::weightUpdate>(this),
                              weight_iterator_t::notify_touch::from<sample_set_t, &sample_set_t::weightStatisticReset>(this));
    }

    inline state_iterator_t getStateIterator()
    {
        return state_iterator_t(*p_t_1_,
                              state_iterator_t::notify_touch::from<sample_set_t, &sample_set_t::resetIndexationStatistic>(this));
    }

    inline sample_insertion_t getInsertion()
    {
        weightStatisticReset();
        p_t_1_density_->clear();
        p_t_->clear();
        return SampleInsertion<StateT>(*p_t_,
                                       SampleInsertion::notify_update::from<sample_set_t, &SampleSet::insertionUpdate>(this),
                                       SampleInsertion::notify_closed::from<sample_set_t, &SampleSet::insertionClosed>(this));
    }


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

    inline std::string const & getFrameID() const
    {
        return frame_id_;
    }

    inline double getMaximumWeight() const
    {
        return maximum_weight_;
    }

    inline double getAverageWeight() const
    {
        return average_weight_;
    }

    inline double getWeightSum() const
    {
        return weight_sum_;
    }

    inline bool isNormalized() const
    {
        return weight_sum_ == 1.0;
    }

    inline sample_vector_t const & getSamples() const
    {
        return *p_t_1_;
    }

private:
    std::string             frame_id_;
    std::size_t             minimum_sample_size_;
    std::size_t             maximum_sample_size_;

    double                  maximum_weight_;
    double                  average_weight_;
    double                  weight_sum_;

    sample_vector_t::Ptr    p_t_1_;
    sample_density_t::Ptr   p_t_1_density_;
    sample_vector_t::Ptr    p_t_;

    void weightStatisticReset()
    {
        maximum_weight_ = 0.0;
        average_weight_ = 0.0;
        weight_sum_     = 0.0;
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
        weightUpdate(sample.weight);
        p_t_1_density_->insert(sample);
    }

    void insertionClosed()
    {
        std::swap(p_t_, p_t_1_);
        p_t_1_density_->cluster();
    }
};
}


#endif // SAMPLE_SET_HPP
