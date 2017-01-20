#ifndef INDEXATION_DATA_HPP
#define INDEXATION_DATA_HPP

#include "particle.hpp"

#include <vector>

namespace muse_amcl {
struct IndexationData {
    using ParticlePtrs = std::vector<const Particle*>;

    int       cluster_ = -1;
    ParticlePtrs samples_;
    double    weight_;

    IndexationData() :
        weight_(0.0)
    {
    }

    IndexationData(const Particle &sample)
    {
        samples_.emplace_back(&sample);
        weight_ = sample.weight_;
    }

    inline void merge(const IndexationData &other)
    {
        samples_.insert(samples_.end(), other.samples_.begin(), other.samples_.end());
        weight_ += other.weight_;
    }
};
}
#endif // INDEXATION_DATA_HPP
