#ifndef KLD_DATA_HPP
#define KLD_DATA_HPP

#include <muse_amcl/particle_filter/particle.hpp>

namespace muse_amcl {
struct Data : public kdtree::KDTreeNodeClusteringSupport
{
    std::vector<const Particle*> samples;
    double weight;

    inline void merge(Data&& other)
    {
        this->samples.insert(this->samples.end(),
                             other.samples.begin(),
                             other.samples.end());
        this->weight += other.weight;
    }

    static inline Data create(const Particle& pt)
    {
        Data data;
        data.samples.emplace_back(&pt);
        data.weight = pt.weight_;
        return data;
    }
};
}

#endif // KLD_DATA_HPP
