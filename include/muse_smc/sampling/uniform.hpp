#ifndef UNIFORM_HPP
#define UNIFORM_HPP

#include <memory>
#include <vector>

#include <muse_smc/samples/sample_set.hpp>

namespace muse_smc {
template<typename state_space_description_t>
class UniformSampling {
public:
    using Ptr          = std::shared_ptr<UniformSampling>;
    using sample_t     = typename state_space_description_t::sample_t;
    using sample_set_t = SampleSet<state_space_description_t>;

    inline UniformSampling()
    {
    }

    virtual ~UniformSampling() = default;

    virtual bool apply(sample_set_t &sample_set) = 0;
    virtual void apply(sample_t &sample) = 0;
    virtual bool update(const std::string &frame) = 0;
};
}

#endif // UNIFORM_HPP
