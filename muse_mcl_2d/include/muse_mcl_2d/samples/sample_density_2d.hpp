#ifndef SAMPLE_DENSITY_2D_HPP
#define SAMPLE_DENSITY_2D_HPP

#include <muse_smc/samples/sample_density.hpp>

#include <muse_mcl_2d/samples/sample_indexation_2d.hpp>
#include <muse_mcl_2d/samples/sample_density_data_2d.hpp>

namespace muse_mcl_2d {
class SampleDensity2D : public SampleDensity2D<Sample2D>
{
public:
    using indexation_t = SampleIndexation2D;

    SampleDensity2D(const indexation_t &indexation) :
        indexation_(indexation)
    {
    }

    virtual void clear() override
    {
    }

    virtual void insert(const Sample2D &sample) override
    {

    }

    virtual void estimate() override
    {

    }

protected:
    indexation_t indexation_;

};
}

#endif // SAMPLE_DENSITY_2D_HPP
