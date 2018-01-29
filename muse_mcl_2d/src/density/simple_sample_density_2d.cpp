#include <ros/ros.h>
#include <unordered_map>

#include <muse_mcl_2d/density/sample_density_2d.hpp>
#include "sample_indexation_2d.hpp"
#include "sample_clustering_2d.hpp"

#include <cslibs_indexed_storage/storage.hpp>
#include <cslibs_indexed_storage/backend/kdtree/kdtree_buffered.hpp>

namespace cis = cslibs_indexed_storage;

namespace muse_mcl_2d {
class SimpleSampleDensity2D : public muse_mcl_2d::SampleDensity2D
{
public:
    using indexation_t              = SampleIndexation2D;
    using sample_data_t             = SampleDensityData2D;
    using clustering_t              = SampleClustering2D;

    using index_t                   = indexation_t::index_t;

    using cluster_map_t             = clustering_t::cluster_map_t;
    using distribution_map_t        = clustering_t::distribution_map_t;
    using angular_mean_map_t        = clustering_t::angular_mean_map_t;

    using cis_kd_tree_buffered_t    = cis::Storage<sample_data_t, index_t, cis::backend::kdtree::KDTreeBuffered>;
    using cis_kd_tree_clustering_t  = cis::operations::clustering::Clustering<cis_kd_tree_buffered_t>;

    virtual void setup(ros::NodeHandle &nh) override
    {
        auto param_name = [this](const std::string &name){return name_ + "/" + name;};

        const double resolution_linear  = nh.param<double>(param_name("resolution_linear"), 0.1);
        const double resolution_angular = cslibs_math::common::angle::toRad(nh.param<double>(param_name("resolution_angular"), 5.0));
        const std::size_t maximum_sample_size = static_cast<std::size_t>(nh.param<int>(param_name("maximum_sample_size"), 0));

        indexation_ = {{resolution_linear, resolution_angular}};
        kdtree_.reset(new cis_kd_tree_buffered_t);
        kdtree_->set<cis::option::tags::node_allocator_chunk_size>(2 * maximum_sample_size + 1);

        clustering_impl_ = clustering_t(indexation_);

    }

    virtual void clear() override
    {
        clustering_impl_.clear();
        kdtree_->clear();
    }

    virtual void insert(const Sample2D &sample) override
    {
        kdtree_->insert(indexation_.create(sample), sample_data_t(sample));
    }

    virtual void estimate() override
    {
        cis_kd_tree_clustering_t clustering(*kdtree_);
        clustering.cluster(clustering_impl_);
    }

    cluster_map_t const & clusters() const
    {
        return clustering_impl_.clusters;
    }

    distribution_map_t const & clusterDistributions() const
    {
        return clustering_impl_.distributions;
    }

    angular_mean_map_t const & clusterAngularMeans() const
    {
        return clustering_impl_.angular_means;
    }

    std::size_t histogramSize() const
    {
        return kdtree_->size();
    }

protected:
    indexation_t                            indexation_;
    std::shared_ptr<cis_kd_tree_buffered_t> kdtree_;

    clustering_t                            clustering_impl_;
};
}

CLASS_LOADER_REGISTER_CLASS(muse_mcl_2d::SimpleSampleDensity2D, muse_mcl_2d::SampleDensity2D)
