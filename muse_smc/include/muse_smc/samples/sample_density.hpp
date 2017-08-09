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

    virtual void clear() = 0;
    virtual void insert(const sample_t &sample) = 0;
    virtual void estimate()  = 0;
};
}
#endif // SAMPLE_DENSITY_HPP
