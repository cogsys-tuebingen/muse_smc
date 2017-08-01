#ifndef SAMPLE_DENSITY_HPP
#define SAMPLE_DENSITY_HPP

#include <muse_mcl/particle_filter_v2/sample.hpp>

/// interface

namespace muse_mcl {
template<typename StateT>
class SampleDensity
{
public:
    using Ptr               = std::shared_ptr<SampleDensity<StateT>>;
    using ConstPtr          = std::shared_ptr<SampleDensity<StateT> const>;
    using sample_t          = Sample<StateT>;

    SampleDensity() = delete;

    virtual void clear() = 0;
    virtual void insert(const sample_t &sample) = 0;
    virtual void cluster()  = 0;



};

}
#endif // SAMPLE_DENSITY_HPP
