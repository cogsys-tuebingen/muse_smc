#ifndef MUSE_SMC_TRAITS_SAMPLE_SET_HPP
#define MUSE_SMC_TRAITS_SAMPLE_SET_HPP

#include <muse_smc/smc/traits/sample.hpp>
#include <muse_smc/samples/sample_set.hpp>

namespace muse_smc {
namespace traits {
/**
 * @brief Defines the data type the filter is working with.
 * @tparam Hypothesis_T   sample type to set the trait for
 */
template<typename Hypothesis_T>
struct SampleSet {
    using sample_t = typename traits::Sample<Hypothesis_T>::type;
    using state_t = typename traits::State<Hypothesis_T>::type;
    using weight_t = double;
    using time_t = typename traits::Time<Hypothesis_T>::type;
    using type = muse_smc::SampleSet<sample_t, state_t, weight_t, time_t>;
    /// put a template switch here ....
};
}
}

#endif // MUSE_SMC_TRAITS_SAMPLE_SET_HPP
