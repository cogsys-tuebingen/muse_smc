#ifndef POSE_GENERATION_NORMAL_HPP
#define POSE_GENERATION_NORMAL_HPP

#include <memory>
#include <vector>
#include <map>

#include <muse_smc/samples/sample_set.hpp>

namespace muse_smc {
template<typename sample_t>
class SamplingNormal {
public:
    typedef std::shared_ptr<SamplingNormal> Ptr;
    using   sample_set_t = SampleSet<sample_t>;


    SamplingNormal() = default;
    virtual ~SamplingNormal() = default;


    inline const static std::string Type()
    {
        return "muse::SamplingNormal";
    }

    inline std::string getName() const
    {
        return name_;
    }

    void setName(const std::string &name)
    {
        name_ = name;
    }

    inline std::size_t getId() const
    {
        return id_;
    }

    inline void setId(const std::size_t id)
    {
        id_ = id;
    }

    virtual void apply(const typename sample_t::state_t      &pose,
                       const typename sample_t::covariance_t &covariance,
                       sample_set_t::sample_insertion_t      &insertion) = 0;
    virtual void update() = 0;

protected:
    std::string name_;
    std::size_t id_;
};
}
#endif // POSE_GENERATION_NORMAL_HPP
