#ifndef CLUSTER_DATA_HPP
#define CLUSTER_DATA_HPP

#include <vector>

#include "particle.hpp"

namespace muse_mcl {
namespace clustering {
template<typename StateT>
struct Data {
    using particle_t = Particle<StateT>;

    int cluster_id = -1;
    std::vector<particle_t*> samples;

    inline Data() = default;

    inline Data(const Data &other) :
        cluster_id(other.cluster_id),
        samples(other.samples)
    {
    }

    inline Data(Data &&other) :
        cluster_id(other.cluster_id),
        samples(std::move(other.samples))
    {
    }

    inline Data(const particle_t &sample)
    {
        samples.emplace_back(&sample);
    }

    inline virtual ~Data()
    {
    }

    virtual inline Data & operator = (const Data &other)
    {
        cluster_id = other.cluster_id;
        samples = other.samples;
    }

    virtual inline void merge(const Data &other)
    {
        samples.insert(samples.end(), other.samples.begin(), other.samples.end());
    }


};
}
}
#endif // CLUSTER_DATA_HPP
