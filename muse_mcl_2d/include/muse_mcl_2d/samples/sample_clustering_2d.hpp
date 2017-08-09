#ifndef SAMPLE_CLUSTERING_2D_HPP
#define SAMPLE_CLUSTERING_2D_HPP

#include <unordered_map>

#include <cslibs_indexed_storage/operations/clustering.hpp>

#include <muse_mcl_2d/samples/sample_density_data_2d.hpp>
#include <muse_mcl_2d/samples/sample_indexation_2d.hpp>

namespace cis = cslibs_indexed_storage;

namespace muse_mcl_2d {
struct SampleClustering2D {
    using indexation_t = SampleIndexation2D;
    using index_t = SampleIndexation2D::index_t;
    using allocator_t = Eigen::aligned_allocator<std::pair<const int, SampleDensityData2D::distribution_t>>;
    using distribution_map_t = std::unordered_map<int,
                                                  SampleDensityData2D::distribution_t, std::hash<int>, std::equal_to<int>,
                                                  allocator_t>;


    /// required definitions -->
    using neighborhood_t  = cis::operations::clustering::GridNeighborhoodStatic<std::tuple_size<index_t::Base>::value, 3>;
    using visitor_index_t = neighborhood_t::offset_t;

    SampleClustering2D(indexation_t &indexation) :
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
    inline bool start(const index_t &, SampleDensityData2D& data)
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
    bool extend(const index_t&, const index_t&, SampleDensityData2D& data)
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
    void visit_neighbours(const index_t&, const visitor_t& visitor)
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
    index_t add(const index_t& a, const offset_t& b) const
    {
        return index_t({a[0] + b[0], a[1] + b[1], (a[2] + b[2]) % angular_bins_});
    }

    indexation_t indexation_;
    int          current_cluster_ = -1;
    int          angular_bins_;
    std::unordered_map<int, SampleDensityData2D::sample_ptr_vector_t> clusters_;
    distribution_map_t distributions_;
    std::unordered_map<int, SampleDensityData2D::angular_mean_t>  angular_means_;

};

}


#endif // SAMPLE_CLUSTERING_2D_HPP
