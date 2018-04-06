#ifndef SAMPLE_DENSITY_HPP
#define SAMPLE_DENSITY_HPP

#include <memory>

namespace muse_smc {
template<typename sample_t>
class SampleDensity
{
public:
    using sample_density_t  = SampleDensity<sample_t>;
    using Ptr               = std::shared_ptr<sample_density_t>;
    using ConstPtr          = std::shared_ptr<sample_density_t const>;

    inline const static std::string Type()
    {
        return "muse_smc::SampleDensity";
    }

    inline void setName(const std::string &name)
    {
        name_ = name;
    }

    inline std::string getName() const
    {
        return name_;
    }

    inline void setId(const std::size_t id)
    {
        id_ = id;
    }

    inline std::size_t getId() const
    {
        return id_;
    }

    virtual void clear() = 0;
    virtual void insert(const sample_t &sample) = 0;
    virtual void estimate()  = 0;

protected:
    std::string name_;
    std::size_t id_;

};
}
#endif // SAMPLE_DENSITY_HPP
