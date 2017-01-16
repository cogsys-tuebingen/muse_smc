#ifndef CLUSTERING_HPP
#define CLUSTERING_HPP

#include <cslibs_indexed_storage/storage.hpp>
#include <cslibs_indexed_storage/backend/kdtree/kdtree.hpp>
#include <cslibs_indexed_storage/backend/array/array.hpp>

#include "clustering_operation.hpp"
#include "particle.hpp"

namespace muse_amcl {
namespace clustering {

namespace cis = cslibs_indexed_storage;

template<typename Storage>
void create(const Indexation &indexation,
            const std::vector<Particle> &particles)
{
    Storage store;
    for(const auto &sample : particles)
    {
        store.insert(indexation.create(sample), Data(sample));
    }
}

template<typename Storage>
void cluster(const Indexation &indexation,
             const std::vector<Particle> &particles)
{
    Storage store;
    for(const auto &sample : particles) {
        store.insert(indexation.create(sample));
    }

    ClusterinOperation clusters;
    cis::operations::clustering::Clustering<Storage> co(store);
    co.cluster(clusters);
}

using KDTree = cis::Storage<Data, Indexation::Index, cis::backend::kdtree::KDTree>;
using Array  = cis::Storage<Data, Indexation::Index, cis::backend::array::Array>;

}
}

#endif // CLUSTERING_HPP
