#ifndef SAMPLE_DENSITY_DATA_2D_HPP
#define SAMPLE_DENSITY_DATA_2D_HPP

#include <muse_mcl_2d/samples/sample_2d.hpp>

#include <muse_smc/math/weighted_distribution.hpp>
#include <muse_smc/math/weighted_angular_mean.hpp>

#include <vector>

namespace muse_mcl_2d {
struct SampleDensityData2D {
    using sample_ptr_vector_t = std::vector<const Sample2D *>;
    using distribution_t      = muse_smc::math::statistic::WeightedDistribution<2>;
    using angular_mean_t      = muse_smc::math::statistic::WeightedAngularMean;

    int                 cluster = -1;
    sample_ptr_vector_t samples;
    distribution_t      distribution;
    angular_mean_t      angular_mean;

    SampleDensityData2D() = default;
    SampleDensityData2D(const SampleDensityData2D &other) :
        cluster(other.cluster),
        samples((other.samples)),
        distribution((other.distribution)),
        angular_mean((other.angular_mean))
    {
    }


    SampleDensityData2D(SampleDensityData2D &&other) :
        cluster(other.cluster),
        samples(std::move(other.samples)),
        distribution(std::move(other.distribution)),
        angular_mean(std::move(other.angular_mean))
    {
    }

    SampleDensityData2D& operator = (const SampleDensityData2D &other)
    {
        cluster         = (other.cluster);
        samples         = ((other.samples));
        distribution    = ((other.distribution));
        angular_mean    = ((other.angular_mean));
        return *this;
    }

    virtual ~SampleDensityData2D()
    {
    }


    SampleDensityData2D(const Sample2D &sample)
    {
        samples.emplace_back(&sample);
        distribution.add(sample.state.translation().toEigen(), sample.weight);
        angular_mean.add(sample.state.yaw(), sample.weight);
     }

    inline void merge(const SampleDensityData2D &other)
    {
        samples.insert(samples.end(), other.samples.begin(), other.samples.end());
        distribution += other.distribution;
        angular_mean += other.angular_mean;
    }

};
}

#endif // SAMPLE_DENSITY_DATA_2D_HPP