#ifndef CLUSTERING_OPERATION_HPP
#define CLUSTERING_OPERATION_HPP

#include <cslibs_indexed_storage/operations/clustering.hpp>

#include "indexation.hpp"
#include "clustering_data.hpp"

#include <unordered_map>

namespace cis = cslibs_indexed_storage;

namespace muse_mcl {
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

    ClusteringImpl(Indexation &indexation) :
        indexation_(indexation)
    {
        auto resolution = indexation_.getResolution();
        angular_bins_ = std::floor(2 * M_PI / resolution[2]);
    }

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
        distributions_.emplace(current_cluster_, data.distribution_);
        angular_means_.emplace(current_cluster_, data.angular_mean_);

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

        auto& cluster      = clusters_[current_cluster_];
        auto& distribution = distributions_[current_cluster_];
        auto& angular_mean = angular_means_[current_cluster_];

        cluster.insert(cluster.end(), data.samples_.begin(), data.samples_.end());
        distribution += data.distribution_;
        angular_mean += data.angular_mean_;

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

    /**
     * @brief add enables the clustering to deal with angle wrapping.
     * @param a - the first index
     * @param b - the second index
     * @return
     */
    template<typename offset_t>
    Index add(const Index& a, const offset_t& b) const
    {
        return Index({a[0] + b[0], a[1] + b[1], (a[2] + b[2]) % angular_bins_});
    }

    Indexation                                 &indexation_;
    int current_cluster_ = -1;
    int angular_bins_;
    std::unordered_map<int, Data::ParticlePtrs> clusters_;
    using Distributions = std::unordered_map<int, Data::Distribution, std::hash<int>, std::equal_to<int>, Eigen::aligned_allocator<std::pair<const int, Data::Distribution>>>;
    Distributions distributions_;
    std::unordered_map<int, Data::AngularMean>  angular_means_;

};
}
}

#endif // CLUSTERING_OPERATION_HPP
