#ifndef CLUSTERING_DATA_HPP
#define CLUSTERING_DATA_HPP

#include <muse_amcl/math/weighted_distribution.hpp>
#include <muse_amcl/particle_filter/particle.hpp>
#include <vector>

namespace muse_amcl {
namespace clustering {
struct Data {
    using ParticlePtrs = std::vector<const Particle*>;

    int                                      cluster_ = -1;
    ParticlePtrs                             samples_;
    math::statistic::WeightedDistribution<3> distribution_;    /// @TODO remove fixed dimension
                                                       /// @TODO check what kind of sum
                                                       /// that should be
    Data()
    {
    }

    Data(const Particle &sample)
    {
        samples_.emplace_back(&sample);
        distribution_.add(sample.pose_.eigen3D(), sample.weight_);
    }

    inline void merge(const Data &other)
    {
        samples_.insert(samples_.end(), other.samples_.begin(), other.samples_.end());
        distribution_ += other.distribution_;
    }
};
}
}
#endif // CLUSTERING_DATA_HPP
