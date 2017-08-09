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
                                                  SampleDensityData2D::distribution_t,
                                                  std::hash<int>,
                                                  std::equal_to<int>,
                                                  allocator_t>;
    using cluster_map_t = std::unordered_map<int, SampleDensityData2D::sample_ptr_vector_t>;
    using angular_mean_map_t = std::unordered_map<int, SampleDensityData2D::angular_mean_t>;

    /// required definitions -->
    using neighborhood_t  = cis::operations::clustering::GridNeighborhoodStatic<std::tuple_size<index_t::base_t>::value, 3>;
    using visitor_index_t = neighborhood_t::offset_t;

    SampleClustering2D(indexation_t &indexation) :
        indexation(indexation)
    {
        auto resolution = indexation.getResolution();
        angular_bins = std::floor(2 * M_PI / resolution[2]);
    }

    inline void clear()
    {
        current_cluster = -1;
        clusters.clear();
        distributions.clear();
        angular_means.clear();
    }

    inline bool start(const index_t &, SampleDensityData2D& data)
    {
        if(data.cluster != -1)
            return false;

        ++current_cluster;

        clusters.emplace(current_cluster, data.samples);
        distributions.emplace(current_cluster, data.distribution);
        angular_means.emplace(current_cluster, data.angular_mean);

        data.cluster = current_cluster;

        return true;
    }

    bool extend(const index_t&, const index_t&, SampleDensityData2D& data)
    {
        if (data.cluster != -1)
            return false;

        auto& cluster      = clusters[current_cluster];
        auto& distribution = distributions[current_cluster];
        auto& angular_mean = angular_means[current_cluster];

        cluster.insert(cluster.end(), data.samples.begin(), data.samples.end());
        distribution += data.distribution;
        angular_mean += data.angular_mean;

        data.cluster = current_cluster;
        return true;
    }


    template<typename visitor_t>
    void visit_neighbours(const index_t&, const visitor_t& visitor)
    {
        static constexpr auto neighborhood = neighborhood_t{};
        neighborhood.visit(visitor);
    }

    template<typename offset_t>
    index_t add(const index_t& a, const offset_t& b) const
    {
        return index_t({a[0] + b[0], a[1] + b[1], (a[2] + b[2]) % angular_bins});
    }

    indexation_t       indexation;
    int                current_cluster = -1;
    int                angular_bins;
    cluster_map_t      clusters;
    distribution_map_t distributions;
    angular_mean_map_t angular_means;

};

}


#endif // SAMPLE_CLUSTERING_2D_HPP
