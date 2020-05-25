#ifndef MUSE_SMC_TRAITS_SAMPLE_SET_HPP
#define MUSE_SMC_TRAITS_SAMPLE_SET_HPP

#include <muse_smc/smc/traits/sample.hpp>
#include <muse_smc/samples/sample_set.hpp>

namespace muse_smc {
namespace traits {
/**
 * @brief Defines the data type the filter is working with.
 * @tparam Sample_T   sample type to set the trait for
 */
template<typename Sample_T>
struct SampleSet {
    using time_t = typename traits::Time<Sample_T>::type;
    using type = muse_smc::SampleSet<Sample_T, time_t>;
    /// put a template switch here ....
};
}
}

#endif // MUSE_SMC_TRAITS_SAMPLE_SET_HPP
