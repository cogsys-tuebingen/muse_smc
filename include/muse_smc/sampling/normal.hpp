#ifndef NORMAL_HPP
#define NORMAL_HPP

#include <memory>
#include <vector>
#include <map>

#include <muse_smc/samples/sample_set.hpp>

namespace muse_smc {
template<typename sample_t>
class NormalSampling
{
public:
    using Ptr          = std::shared_ptr<NormalSampling>;
    using state_t      = typename traits::State<sample_t>::type;
    using covariance_t = typename traits::Covariance<sample_t>::type;
    using sample_set_t = SampleSet<sample_t>;

    inline NormalSampling() = default;
    virtual ~NormalSampling() = default;

    virtual bool apply(const state_t             &state,
                       const covariance_t        &covariance,
                       sample_set_t              &sample_set) = 0;
    virtual bool update(const std::string &frame) = 0;
};
}

#endif // NORMAL_HPP
