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

    /// laser to base transform
    tf::Transform laser_to_base;
    tf_provider_->lookupTransform(robot_base_frame_,
                                  laser_data.getFrame(),
                                  laser_data.getTimeFrame().end,
                                  laser_to_base,
                                  tf_timeout_);

    const LaserScan2D::Rays rays = laser_data.getRays();
    const ParticleSet::Weights::iterator end = set.end();
    const std::size_t rays_size = rays.size();
    const std::size_t ray_step      = (rays_size) / max_beams_;
    for(auto it = set.begin() ; it != end ; ++it) {
        const math::Pose pose = laser_to_base * it.getData().pose_; /// we take off from the laser pose
        for(std::size_t i = 0 ; i < rays_size ;  i+= ray_step) {
            //// here comes the grid map into play
//            const double z_bar;

        }
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
    lambda_short_ = nh_private.param(privateParameter("lambda_short"), 0.01);
    chi_outlier_  = nh_private.param(privateParameter("chi_outlier"), 0.05);
    range_min_    = nh_private.param(privateParameter("range_min"), 0.05);
    range_max_    = nh_private.param(privateParameter("range_max"), 30.0);
}
