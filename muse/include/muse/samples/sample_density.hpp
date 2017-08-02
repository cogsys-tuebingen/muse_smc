#ifndef SAMPLE_DENSITY_HPP
#define SAMPLE_DENSITY_HPP

#include <memory>

namespace muse {
template<typename sample_t>
class SampleDensity
{
public:
    using sample_density_t  = SampleDensity<sample_t>;
    using Ptr               = std::shared_ptr<sample_density_t>;
    using ConstPtr          = std::shared_ptr<sample_density_t const>;

    SampleDensity() = delete;

    virtual void clear() = 0;
    virtual void insert(const sample_t &sample) = 0;
    virtual void cluster()  = 0;
};
}
#endif // SAMPLE_DENSITY_HPP
