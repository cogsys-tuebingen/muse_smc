#ifndef CLUSTERING_DATA_HPP
#define CLUSTERING_DATA_HPP

#include "particle.hpp"
#include <vector>

namespace muse_amcl {
namespace clustering {
struct Data {
    using Particles = std::vector<const Particle*>;

    int cluster_ = -1;
    Particles samples_;
    double weight;

    Data() :
        weight(0.0)
    {
    }

    Data(const Particle &sample)
    {
        samples_.emplace_back(&sample);
        weight = sample.weight_;
    }

    inline void merge(const Data &other)
    {
        samples_.insert(samples_.end(), other.samples_.begin(), other.samples_.end());
        weight += other.weight;
    }
};
}
}
#endif // CLUSTERING_DATA_HPP
