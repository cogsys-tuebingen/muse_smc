#ifndef SAMPLE_DENSITY_2D_HPP
#define SAMPLE_DENSITY_2D_HPP

#include <unordered_map>

#include <muse_smc/samples/sample_density.hpp>

#include <muse_mcl_2d/samples/sample_indexation_2d.hpp>
#include <muse_mcl_2d/samples/sample_density_data_2d.hpp>
#include <muse_mcl_2d/samples/sample_clustering_2d.hpp>

#include <cslibs_indexed_storage/storage.hpp>
#include <cslibs_indexed_storage/backend/kdtree/kdtree_buffered.hpp>

namespace cis = cslibs_indexed_storage;

namespace muse_mcl_2d {
class SampleDensity2D : public muse_smc::SampleDensity<Sample2D>
{
public:
    using Ptr = std::shared_ptr<SampleDensity2D>;
    using ConstPtr = std::shared_ptr<SampleDensity2D const>;

    using indexation_t              = SampleIndexation2D;
    using sample_data_t             = SampleDensityData2D;
    using clustering_t              = SampleClustering2D;

    using index_t                   = indexation_t::index_t;

    using cluster_map_t             = clustering_t::cluster_map_t;
    using distribution_map_t        = clustering_t::distribution_map_t;
    using angular_mean_map_t        = clustering_t::angular_mean_map_t;

    using cis_kd_tree_buffered_t    = cis::Storage<sample_data_t, index_t, cis::backend::kdtree::KDTreeBuffered>;
    using cis_kd_tree_clustering_t  = cis::operations::clustering::Clustering<cis_kd_tree_buffered_t>;


    inline SampleDensity2D(const indexation_t &indexation,
                           const std::size_t maximum_sample_size) :
        indexation_(indexation),
        kdtree_(new cis_kd_tree_buffered_t),
        clustering_impl_(indexation_)
    {
        kdtree_->set<cis::option::tags::node_allocator_chunk_size>(2 * maximum_sample_size + 1);
    }

    inline virtual void clear() override
    {
        clustering_impl_.clear();
        kdtree_->clear();
    }

    inline virtual void insert(const Sample2D &sample) override
    {
        kdtree_->insert(indexation_.create(sample), sample_data_t(sample));
    }

    inline virtual void estimate() override
    {
        cis_kd_tree_clustering_t clustering(*kdtree_);
        clustering.cluster(clustering_impl_);
    }

    inline cluster_map_t const & clusters() const
    {
        return clustering_impl_.clusters;
    }

    inline distribution_map_t const & clusterDistributions() const
    {
        return clustering_impl_.distributions;
    }

    inline angular_mean_map_t const & clusterAngularMeans() const
    {
        return clustering_impl_.angular_means;
    }

    inline std::size_t histogramSize() const
    {
        return kdtree_->size();
    }

protected:
    indexation_t                            indexation_;
    std::shared_ptr<cis_kd_tree_buffered_t> kdtree_;

    clustering_t                            clustering_impl_;
};
}

#endif // SAMPLE_DENSITY_2D_HPP
