#ifndef SAMPLE_SET_HPP
#define SAMPLE_SET_HPP

#include <string>

#include <muse_mcl/utility/buffered_vector.hpp>
#include <muse_mcl/particle_filter_v2/sample.hpp>

namespace muse_mcl {
template<typename StateT>
class SampleSet
{
public:
    using Ptr      = std::shared_ptr<SampleSet<StateT>>;
    using ConstPtr = std::shared_ptr<SampleSet<StateT> const>;

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
              const std::size_t sample_size) :
        frame_id_(frame_id)
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
              const std::size_t sample_size_maxmimum) :
        frame_id_(frame_id)
    {
    }

    /**
     * @brief   Move assignment operator
     * @param   other - the particle set to be moved
     */
    SampleSet& operator = (SampleSet &&other) = default;


    /**
     * @brief getMinimumSampleSize returns the minimum  of allowed sample sizes.
     * @return  - minimum_sample_size_
     */
    std::size_t getMinimumSampleSize() const
    {
        return minimum_sample_size_;
    }

    /**
     * @brief getMaximumSampleSize return the maximum of allowed sample sizes.
     * @return  - maximum_sample_size_
     */
    std::size_t getMaximumSampleSize() const
    {
        return maximum_sample_size_;
    }

    /**
     * @brief getSampleSize returns the current sample size of the set.
     * @return  - size of current set p_t_1_
     */
    std::size_t getSampleSize() const
    {
         return p_t_1_->size();
    }

private:
    std::string frame_id_;
    std::size_t minimum_sample_size_;
    std::size_t maximum_sample_size_;

    typename std::buffered_vector<typename Sample<StateT>>::Ptr p_t_1_;
    typename std::buffered_vector<typename Sample<StateT>>::Ptr p_t_;
};
}


#endif // SAMPLE_SET_HPP
