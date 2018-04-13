#ifndef NORMAL_HPP
#define NORMAL_HPP

#include <memory>
#include <vector>
#include <map>

#include <muse_smc/samples/sample_set.hpp>

namespace muse_smc {
template<typename state_space_description_t>
class NormalSampling
{
public:
    using Ptr          = std::shared_ptr<NormalSampling>;
    using sample_t     = typename state_space_description_t::sample_t;
    using state_t      = typename state_space_description_t::state_t;
    using covariance_t = typename state_space_description_t::covariance_t;
    using sample_set_t = SampleSet<state_space_description_t>;

    inline NormalSampling()
    {
    }

    virtual ~NormalSampling()
    {
    }

    virtual void apply(const state_t             &state,
                       const covariance_t        &covariance,
                       sample_set_t              &sample_set) = 0;
    virtual void update(const std::string &frame) = 0;
};
}

#endif // NORMAL_HPP
