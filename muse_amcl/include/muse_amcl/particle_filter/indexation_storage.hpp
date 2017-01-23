#ifndef INDEXED_STORAGE_HPP
#define INDEXED_STORAGE_HPP

#include <memory>
#include <limits>

#include "indexation.hpp"
#include "indexation_data.hpp"

#include <cslibs_indexed_storage/storage.hpp>
#include <cslibs_indexed_storage/backend/kdtree/kdtree_buffered.hpp>
#include <cslibs_indexed_storage/backend/array/array.hpp>

namespace cis = cslibs_indexed_storage;

/**
 * @brief The IndexationStorage class is used to index samples so the particle
 *        density can be estimated. It is also useful to build up and calculate
 *        the histogram size.
 */
namespace muse_amcl {
class IndexationStorage {
public:
    using IndexType = Indexation::IndexType;
    using SizeType  = Indexation::SizeType;
    using VolumeType = std::array<double, 3>;

    IndexationStorage(const std::size_t tree_nodes_size,
                      const Indexation &indexation) :
        min_index_(std::numeric_limits<int>::max()),
        max_index_(std::numeric_limits<int>::min()),
        indexation_(indexation)
    {
    }

    IndexationStorage(const std::size_t tree_nodes_size,
                      const VolumeType &array_volume,
                      const Indexation &indexation) :
        min_index_(std::numeric_limits<int>::max()),
        max_index_(std::numeric_limits<int>::min()),
        indexation_(indexation)
    {
    }

    /**
     * @brief resetLimits resets the limits in order find new limits
     *        for the spatial extent of the particle set.
     */
    inline void resetLimits()
    {
        min_index_.fill(std::numeric_limits<int>::max());
        max_index_.fill(std::numeric_limits<int>::min());
    }

    /**
     * @brief updateLimits uses a single sample to update the current
     *        limits.
     * @param sample - the particle sample
     */
    inline void updateLimits(const Particle &sample)
    {
        IndexType si = indexation_.create(sample);
        min_index_.min(si);
        max_index_.max(si);
    }

    /**
     * @brief updateLimits
     * @param particles
     */
    inline void updateLimits(const std::vector<Particle> &particles)
    {
        for(const auto &p : particles)
            updateLimits(p);
    }

    
    
    
    
private:
    IndexType  min_index_;
    IndexType  max_index_;
    SizeType   size_;
    Indexation indexation_;

    // using KDTree = cis::Storage<Data, Indexation::IndexType::Base, cis::backend::kdtree::KDTree>;

    using KDTreeBuffered = cis::Storage<IndexationData, Indexation::IndexType::Base, cis::backend::kdtree::KDTreeBuffered>;
    using Array  = cis::Storage<IndexationData, Indexation::IndexType::Base, cis::backend::array::Array>;

    std::shared_ptr<KDTreeBuffered> kdtree_;    /// wide range density estimation
    std::shared_ptr<Array>          array_;     /// near range density estimation
};
}

#endif // INDEXED_STORAGE_HPP
