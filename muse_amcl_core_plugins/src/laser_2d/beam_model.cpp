#include "beam_model.h"

#include <muse_amcl_core_plugins/laser_2d/laser_scan_2d.hpp>
#include <muse_amcl_core_plugins/maps_2d/binary_gridmap.h>

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(muse_amcl::BeamModel, muse_amcl::UpdateModel)

using namespace muse_amcl;


BeamModel::BeamModel()
{

}

void BeamModel::update(const Data::ConstPtr  &data,
                       const Map::ConstPtr  &map,
                       ParticleSet::Weights set)
{

    const maps::BinaryGridMap &gridmap = map->as<maps::BinaryGridMap>();
    const LaserScan2D         &laser_data = data->as<LaserScan2D>();
    const LaserScan2D::Rays   &laser_rays = laser_data.getRays();

    /// laser to base transform
    tf::Transform base_T_laser;
    tf::Transform map_T_world;
    tf_provider_->lookupTransform(robot_base_frame_,
                                  laser_data.getFrame(),
                                  laser_data.getTimeFrame().end,
                                  base_T_laser,
                                  tf_timeout_);
    tf_provider_->lookupTransform(world_frame_,
                                  gridmap.getFrame(),
                                  laser_data.getTimeFrame().end,
                                  map_T_world,
                                  tf_timeout_);

    const LaserScan2D::Rays rays = laser_data.getRays();
    const ParticleSet::Weights::iterator end = set.end();
    const std::size_t rays_size = rays.size();
    const std::size_t ray_step      = (rays_size) / max_beams_;

    /// mixture distribution entries
    auto p_hit = [this](const double z) {
        return z_hit_ * std::exp(-z * z * denominator_hit_);
    };
    auto p_short = [this](const double z, const double ray_range) {
        if(z < 0)
            return z_short_ * lambda_short_ * exp(-lambda_short_ * ray_range);
        return 0.0;
    };
    auto p_max = [this](const double ray_range)
    {
        if(ray_range >= range_max_)
            return z_max_ * 1.0;
        return 0.0;
    };
    auto p_random = [this](const double ray_range)
    {
        if(ray_range < range_max_)
            return p_rand_;
        return 0.0;
    };
    auto probability = [p_hit, p_short, p_max, p_random] (const double ray_range, const double map_range)
    {
        const double z = ray_range - map_range;
        return p_hit(z) + p_short(z, ray_range) + p_max(ray_range) + p_random(ray_range);
    };


    for(auto it = set.begin() ; it != end ; ++it) {
        const math::Pose pose = map_T_world * it.getData().pose_ * base_T_laser; /// laser scanner pose in map coordinates
        double p = 1.0;
        for(std::size_t i = 0 ; i < rays_size ;  i+= ray_step) {
            const double        ray_range = laser_rays[i].range;
            const math::Point   ray_end_point = pose.tf() * laser_rays[i].point;
            const double        map_range = gridmap.getRange(pose.origin(), ray_end_point);
            const double pz = probability(ray_range, map_range);
            p += pz * pz * pz;  /// @todo : fix the inprobable thing ;)
        }
        *it = p;
    }
}

void BeamModel::doSetup(ros::NodeHandle &nh_private)
{
    max_beams_    = nh_private.param(privateParameter("max_beams"), 30);
    z_hit_        = nh_private.param(privateParameter("z_hit"), 0.8);
    z_short_      = nh_private.param(privateParameter("z_short"), 0.1);
    z_max_        = nh_private.param(privateParameter("z_max"), 0.05);
    z_rand_       = nh_private.param(privateParameter("z_rand"), 0.05);
    sigma_hit_    = nh_private.param(privateParameter("sigma_hit"), 0.15);
    denominator_hit_ = 0.5 * 1.0 / (sigma_hit_ * sigma_hit_);
    lambda_short_ = nh_private.param(privateParameter("lambda_short"), 0.01);
    chi_outlier_  = nh_private.param(privateParameter("chi_outlier"), 0.05);
    range_min_    = nh_private.param(privateParameter("range_min"), 0.05);
    range_max_    = nh_private.param(privateParameter("range_max"), 30.0);
    p_rand_       = z_rand_ * 1.0 / range_max_;
}
