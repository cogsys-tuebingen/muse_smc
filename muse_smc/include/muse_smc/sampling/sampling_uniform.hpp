#ifndef POSE_GENERATION_UNIFORM_HPP
#define POSE_GENERATION_UNIFORM_HPP

#include <memory>
#include <vector>

#include <muse_smc/samples/sample_set.hpp>

namespace muse_smc {
template<typename sample_t>
class SamplingUniform {
public:
    typedef std::shared_ptr<SamplingUniform> Ptr;
    using   sample_set_t = SampleSet<sample_t>;

    SamplingUniform() = default;
    virtual ~SamplingUniform() = default;

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

    virtual void apply(typename sample_set_t::sample_insertion_t &insertion) = 0;
    virtual void apply(sample_t &sample) = 0;
    virtual void update() = 0;

protected:
    std::string name_;
    std::size_t id_;
};
}

#endif // POSE_GENERATION_UNIFORM_HPP
