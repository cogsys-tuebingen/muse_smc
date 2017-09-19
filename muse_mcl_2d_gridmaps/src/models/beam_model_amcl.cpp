#include "beam_model_amcl.h"

#include <muse_mcl_2d_laser/laser/laser_2d_scan.hpp>
#include <muse_mcl_2d_gridmaps/static_maps//binary_gridmap.h>

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(muse_mcl_2d_gridmaps::BeamModelAMCL, muse_mcl_2d::UpdateModel2D)

using namespace muse_mcl_2d_gridmaps;

#include <sensor_msgs/PointCloud.h>

BeamModelAMCL::BeamModelAMCL()
{
}

void BeamModelAMCL::apply(const data_t::ConstPtr          &data,
                          const state_space_t::ConstPtr   &map,
                          sample_set_t::weight_iterator_t set)
{
    if(!map->isType<static_maps::BinaryGridMap>()) {
        return;
    }

    const static_maps::BinaryGridMap &gridmap = map->as<static_maps::BinaryGridMap>();
    const muse_mcl_2d_laser::LaserScan2D        &laser_data = data->as<muse_mcl_2d_laser::LaserScan2D>();
    const muse_mcl_2d_laser::LaserScan2D::Rays  &laser_rays = laser_data.getRays();

    /// laser to base transform
    muse_mcl_2d::math::Transform2D b_T_l;
    muse_mcl_2d::math::Transform2D m_T_w;
    if(!tf_->lookupTransform(robot_base_frame_,
                             laser_data.getFrame(),
                             ros::Time(laser_data.getTimeFrame().end.seconds()),
                             b_T_l,
                             tf_timeout_))
        return;
    if(!tf_->lookupTransform(world_frame_,
                             gridmap.getFrame(),
                             ros::Time(laser_data.getTimeFrame().end.seconds()),
                             m_T_w,
                             tf_timeout_))
        return;

    const muse_mcl_2d_laser::LaserScan2D::Rays rays = laser_data.getRays();
    const auto end = set.end();
    const std::size_t rays_size = rays.size();
    const std::size_t ray_step  = std::max(1ul, rays_size / max_beams_);
    const double range_max = laser_data.getRangeMax();
    const double p_rand = z_rand_ * 1.0 / range_max;


    /// mixture distribution entries
    auto p_hit = [this](const double z) {
        return z_hit_ * std::exp(-z * z * denominator_hit_);
    };
    auto p_short = [this](const double z, const double ray_range) {
        if(z < 0)
            return z_short_ * lambda_short_ * exp(-lambda_short_ * ray_range);
        return 0.0;
    };
    auto p_max = [this, range_max](const double ray_range)
    {
        if(ray_range >= range_max)
            return z_max_ * 1.0;
        return 0.0;
    };
    auto p_random = [this, range_max, p_rand](const double ray_range)
    {
        if(ray_range < range_max)
            return p_rand;
        return 0.0;
    };
    auto probability = [p_hit, p_short, p_max, p_random] (const double ray_range, const double map_range)
    {
        const double z = ray_range - map_range;
        return p_hit(z) + p_short(z, ray_range) + p_max(ray_range) + p_random(ray_range);
    };

    for(auto it = set.begin() ; it != end ; ++it) {
        const muse_mcl_2d::math::Pose2D m_T_l = m_T_w * it.getData().state * b_T_l; /// laser scanner pose in map coordinates
        double p = 1.0;
        const muse_mcl_2d::math::Point2D  ray_start_point = m_T_l.translation();
        for(std::size_t i = 0 ; i < rays_size ;  i+= ray_step) {
            const auto &ray = laser_rays[i];
            if(!ray.valid()) {
                p += z_max_;
            } else {
                const double           ray_range = ray.range;
                muse_mcl_2d::math::Point2D   ray_end_point =  m_T_l * ray.point;
                const double map_range = gridmap.getRange(ray_start_point, ray_end_point);
                const double pz = probability(ray_range, map_range);
                p += pz * pz * pz;  /// @todo : fix the inprobable thing ;)
            }
        }
        *it *= p;
    }
}

void BeamModelAMCL::doSetup(ros::NodeHandle &nh)
{
    auto param_name = [this](const std::string &name){return name_ + "/" + name;};

    max_beams_    = nh.param(param_name("max_beams"), 30);
    z_hit_        = nh.param(param_name("z_hit"), 0.8);
    z_short_      = nh.param(param_name("z_short"), 0.1);
    z_max_        = nh.param(param_name("z_max"), 0.05);
    z_rand_       = nh.param(param_name("z_rand"), 0.05);
    sigma_hit_    = nh.param(param_name("sigma_hit"), 0.15);
    denominator_hit_ = 0.5 * 1.0 / (sigma_hit_ * sigma_hit_);
    lambda_short_ = nh.param(param_name("lambda_short"), 0.01);
    chi_outlier_  = nh.param(param_name("chi_outlier"), 0.05);
}
