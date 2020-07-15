#ifndef MUSE_SMC_TRAITS_SAMPLE_DENSITY_HPP
#define MUSE_SMC_TRAITS_SAMPLE_DENSITY_HPP

#include <muse_smc/samples/sample_density.hpp>
#include <muse_smc/smc/traits/sample.hpp>
namespace muse_smc {
namespace traits {
template<typename Hypothesis_T>
struct SampleDensity {
  using sample_t = typename muse_smc::traits::Sample<Hypothesis_T>::type;
  using type = muse_smc::SampleDensity<sample_t>;
};
}
}

#endif  // MUSE_SMC_TRAITS_SAMPLE_HPP
