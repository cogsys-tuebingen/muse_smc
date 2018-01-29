#ifndef SAMPLE_DENSITY_2D_HPP
#define SAMPLE_DENSITY_2D_HPP

#include <ros/ros.h>
#include <unordered_map>
#include <class_loader/class_loader_register_macro.h>

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
    using Ptr                       = std::shared_ptr<SampleDensity2D>;
    using ConstPtr                  = std::shared_ptr<SampleDensity2D const>;

    inline const static std::string Type()
    {
        return "muse_mcl_2d::SampleDensity2D";
    }

    using clustering_t              = SampleClustering2D;
    using cluster_map_t             = clustering_t::cluster_map_t;
    using distribution_map_t        = clustering_t::distribution_map_t;
    using angular_mean_map_t        = clustering_t::angular_mean_map_t;

    virtual void setup(ros::NodeHandle &nh) = 0;
    virtual cluster_map_t const & clusters() const = 0;
    virtual distribution_map_t const & clusterDistributions() const = 0;
    virtual angular_mean_map_t const & clusterAngularMeans() const = 0;
    virtual std::size_t histogramSize() const = 0;
};
}

#endif // SAMPLE_DENSITY_2D_HPP
