#ifndef MUSE_SMC_TRAITS_STATE_SPACE_HPP
#define MUSE_SMC_TRAITS_STATE_SPACE_HPP

#include <muse_smc/state_space/state_space.hpp>
#include <muse_smc/smc/traits/sample.hpp>

namespace muse_smc {
namespace traits {
template<typename Sample_T>
struct StateSpace {
    using state_t                 = typename traits::State<sample_t>::type;
    using state_space_transform_t = typename traits::Transform<sample_t>::type;
    using state_space_boundary_t  = typename traits::StateSpaceBoundary<sample_t>::type;
    using type = StateSpace<state_t, state_space_transform_t, state_space_boundary_t>;
};
}
}
#endif // MUSE_SMC_TRAITS_STATE_SPACE_HPP
