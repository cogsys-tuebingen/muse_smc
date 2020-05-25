#ifndef MUSE_SMC_UNIFORM_SAMPLING_HPP
#define MUSE_SMC_UNIFORM_SAMPLING_HPP

#include <memory>

namespace muse_smc {
template<typename Sample_T, typename SampleSet_T>
class UniformSampling {
public:
    using Ptr          = std::shared_ptr<UniformSampling>;
    using ConstPtr      = std::shared_ptr<UniformSampling const>;

    inline UniformSampling() = default;
    virtual ~UniformSampling() = default;

    virtual bool apply(SampleSet_T &sample_set) = 0;
    virtual void apply(Sample_T &sample) = 0;
    virtual bool update(const std::string &frame) = 0;
};
}

#endif // MUSE_SMC_UNIFORM_SAMPLING_HPP
