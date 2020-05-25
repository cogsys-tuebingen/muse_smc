#ifndef MUSE_SMC_TRAITS_SAMPLE_SET_HPP
#define MUSE_SMC_TRAITS_SAMPLE_SET_HPP

#include <muse_smc/samples/sample_set.hpp>

namespace muse_smc {
namespace traits {
/**
 * @brief Defines the data type the filter is working with.
 * @tparam Sample_T   sample type to set the trait for
 */
template<typename Sample_T>
struct SampleSet {
    using type = SampleSet<Sample_T>;

    /// put a template switch here ....
};
}
}

#endif // MUSE_SMC_TRAITS_SAMPLE_SET_HPP
