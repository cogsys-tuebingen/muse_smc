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
    typedef std::shared_ptr<NormalSampling<sample_t>> Ptr;
    using   sample_set_t = SampleSet<sample_t>;


    NormalSampling() = default;
    virtual ~NormalSampling() = default;


    inline const static std::string Type()
    {
        return "muse_smc::SamplingNormal";
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

    virtual void apply(const typename sample_t::state_t             &state,
                       const typename sample_t::covariance_t        &covariance,
                       sample_set_t                                 &sample_set) = 0;
    virtual void update(const std::string &frame) = 0;

protected:
    std::string name_;
    std::size_t id_;
};
}
#endif // NORMAL_HPP
