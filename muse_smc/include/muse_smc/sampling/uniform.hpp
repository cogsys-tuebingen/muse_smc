#ifndef UNIFORM_HPP
#define UNIFORM_HPP

#include <memory>
#include <vector>

#include <muse_smc/samples/sample_set.hpp>

namespace muse_smc {
template<typename sample_t>
class UniformSampling {
public:
    typedef std::shared_ptr<UniformSampling> Ptr;
    using   sample_set_t = SampleSet<sample_t>;

    UniformSampling() = default;
    virtual ~UniformSampling() = default;

    inline const static std::string Type()
    {
        return "muse_smc::SamplingUniform";
    }

    inline std::string getName() const
    {
        return name_;
    }

    inline void setName(const std::string &name)
    {
        name_  = name;
    }

    inline std::size_t getId() const
    {
        return id_;
    }

    inline void setId(const std::size_t id)
    {
        id_ = id;
    }

    virtual void apply(sample_set_t &sample_set) = 0;
    virtual void apply(sample_t &sample) = 0;
    virtual bool update(const std::string &frame) = 0;

protected:
    std::string name_;
    std::size_t id_;
};
}

#endif // UNIFORM_HPP