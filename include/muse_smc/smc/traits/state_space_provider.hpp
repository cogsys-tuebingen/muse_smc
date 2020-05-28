#ifndef MUSE_SMC_TRAITS_STATE_SPACE_PROVIDER_HPP
#define MUSE_SMC_TRAITS_STATE_SPACE_PROVIDER_HPP

#include <muse_smc/state_space/state_space_provider.hpp>
#include <muse_smc/smc/traits/state_space.hpp>

namespace muse_smc {
namespace traits {
template<typename Hypothesis_T>
struct StateSpaceProvider {
    using type = muse_smc::StateSpaceProvider<typename traits::StateSpace<Hypothesis_T>::type>;
};
}
}

#endif // MUSE_SMC_TRAITS_STATE_SPACE_PROVIDER_HPP
