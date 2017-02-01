#ifndef CLUSTERING_OPERATION_HPP
#define CLUSTERING_OPERATION_HPP

#include <cslibs_indexed_storage/operations/clustering.hpp>

#include "indexation.hpp"
#include "clustering_data.hpp"

#include <unordered_map>

namespace cis = cslibs_indexed_storage;

namespace muse_amcl {
namespace clustering {
/**
 * @brief The ClusteringImpl struct encodes the clustering strategy using the cslibs_index_storage
 *        library.
 */
struct ClusteringImpl {
    using Index = Indexation::IndexType;

    /// required definitions -->
    using neighborhood_t  = cis::operations::clustering::GridNeighborhoodStatic<std::tuple_size<Index::Base>::value, 3>;
    using visitor_index_t = neighborhood_t::offset_t;   //!< currently needed by the clustering API

    /**
     * @brief Open a new cluster if possible.
     * @param data - the data container currently processed
     * @return true indicates whether a cluster could be created
     */
    inline bool start(const Index &, Data& data)
    {
        if(data.cluster_ != -1)
            return false;

        ++current_cluster_;

        clusters_.emplace(current_cluster_, data.samples_);
        data.cluster_ = current_cluster_;

        return true;
    }

    /**
     * @brief Extend an existing cluster with this operation.
     * @param data  - the data wrapper currently in scope
     * @return
     */
    bool extend(const Index&, const Index&, Data& data)
    {
        if (data.cluster_ != -1)
            return false;

        auto& cluster = clusters_[current_cluster_];
        cluster.insert(cluster.end(), data.samples_.begin(), data.samples_.end());

        data.cluster_ = current_cluster_;
        return true;
    }

    /**
     * @brief Visitor for the neighbourhood.
     */
    template<typename visitor_t>
    void visit_neighbours(const Index&, const visitor_t& visitor)
    {
        static constexpr auto neighborhood = neighborhood_t{};
        neighborhood.visit(visitor);
    }

    int current_cluster_ = -1;
    std::unordered_map<int, Data::ParticlePtrs> clusters_;

};
}
}

#endif // CLUSTERING_OPERATION_HPP
