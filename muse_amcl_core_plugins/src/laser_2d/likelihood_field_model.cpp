#include "likelihood_field_model.h"

#include <muse_amcl_core_plugins/laser_2d/laser_scan_2d.hpp>
#include <muse_amcl_core_plugins/maps_2d/distance_gridmap.h>

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(muse_mcl::LikelihoodFieldModel, muse_mcl::UpdateModel)

using namespace muse_mcl;


LikelihoodFieldModel::LikelihoodFieldModel()
{
}

void LikelihoodFieldModel::update(const Data::ConstPtr &data,
                                  const Map::ConstPtr  &map,
                                  ParticleSet::Weights  set)
{
    const maps::DistanceGridMap &gridmap = map->as<maps::DistanceGridMap>();
    const LaserScan2D           &laser_data = data->as<LaserScan2D>();
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
    const std::size_t ray_step  = std::max(1ul, (rays_size) / max_beams_);
    const double range_max = laser_data.getRangeMax();
    const double p_rand = z_rand_ * 1.0 / range_max;

    auto p_hit = [this] (const double z) {
        return z_hit_ * std::exp(-z * z * denominator_hit_);
    };



    for(auto it = set.begin() ; it != end ; ++it) {
        const math::Pose pose = map_T_world * it.getData().pose_ * base_T_laser; /// laser scanner pose in map coordinates
        double p = 0.0;
        for(std::size_t i = 0 ; i < rays_size ;  i+= ray_step) {
            const math::Point   ray_end_point = pose.getPose() * laser_rays[i].point_;
            const double pz = p_hit(gridmap.at(ray_end_point)) + p_rand;
            p += std::log(pz);
        }
        *it *= std::exp(p);
    }
}

void LikelihoodFieldModel::doSetup(ros::NodeHandle &nh_private)
{
    max_beams_ = nh_private.param(privateParameter("max_beams"), 30);
    z_hit_ = nh_private.param(privateParameter("z_hit"), 0.8);
    z_rand_ = nh_private.param(privateParameter("z_rand"), 0.05);
    sigma_hit_ = nh_private.param(privateParameter("sigma_hit"), 0.15);
    denominator_hit_ = 0.5 * 1.0 / (sigma_hit_ * sigma_hit_);

    Logger &l = Logger::getLogger();
    l.info("max_beams_=" + std::to_string(max_beams_), "UpdateModel:" + name_);
    l.info("z_hit_=" + std::to_string(z_hit_), "UpdateModel:" + name_);
    l.info("z_rand_=" + std::to_string(z_rand_), "UpdateModel:" + name_);
    l.info("sigma_hit_=" + std::to_string(sigma_hit_), "UpdateModel:" + name_);

}
