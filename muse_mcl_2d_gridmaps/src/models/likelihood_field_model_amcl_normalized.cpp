#include "likelihood_field_model_amcl_normalized.h"

#include <muse_mcl_2d_laser/laserscan_2d.hpp>

#include <muse_mcl_2d_gridmaps/maps/distance_gridmap.h>

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(muse_mcl_2d_gridmaps::LikelihoodFieldModelAMCLNormalized, muse_mcl_2d::UpdateModel2D)

namespace muse_mcl_2d_gridmaps {
LikelihoodFieldModelAMCLNormalized::LikelihoodFieldModelAMCLNormalized()
{
}

void LikelihoodFieldModelAMCLNormalized::apply(const data_t::ConstPtr   &data,
                                     const state_space_t::ConstPtr      &map,
                                     sample_set_t::weight_iterator_t     set)
{

    if(!map->isType<DistanceGridmap>()) {
        return;
    }
    if(ps_.size() != set.capacity()) {
        ps_.resize(set.capacity());
    }
    std::fill(ps_.begin(), ps_.end(), 0.0);

    const cslibs_gridmaps::static_maps::DistanceGridmap &gridmap = *(map->as<DistanceGridmap>().data());
    const muse_mcl_2d_laser::LaserScan2D             &laser_data = data->as<muse_mcl_2d_laser::LaserScan2D>();
    const muse_mcl_2d_laser::LaserScan2D::rays_t     &laser_rays = laser_data.getRays();

    /// laser to base transform
    cslibs_math_2d::Transform2d b_T_l;
    cslibs_math_2d::Transform2d m_T_w;
    if(!tf_->lookupTransform(robot_base_frame_,
                             laser_data.getFrame(),
                             ros::Time(laser_data.getTimeFrame().end.seconds()),
                             b_T_l,
                             tf_timeout_))
        return;
    if(!tf_->lookupTransform(world_frame_,
                             map->getFrame(),
                             ros::Time(laser_data.getTimeFrame().end.seconds()),
                             m_T_w,
                             tf_timeout_))
        return;

    const muse_mcl_2d_laser::LaserScan2D::rays_t rays = laser_data.getRays();
    const auto end = set.end();
    const auto const_end = set.const_end();
    const std::size_t rays_size = rays.size();
    const std::size_t ray_step  = std::max(1ul, rays_size / max_beams_);
    const double range_max = laser_data.getLinearMax();
    const double p_rand = z_rand_ * 1.0 / range_max;

    auto p_hit = [this] (const double z) {
        return z_hit_ * std::exp(-z * z * denominator_hit_);
    };

    auto it_ps = ps_.begin();
    double p_max = std::numeric_limits<double>::lowest();
    for(auto it = set.const_begin() ; it != const_end ; ++it, ++it_ps) {
        const cslibs_math_2d::Pose2d m_T_l = m_T_w * it->state * b_T_l; /// laser scanner pose in map coordinates
        double p = 1.0;
        for(std::size_t i = 0 ; i < rays_size ;  i+= ray_step) {
            const auto &ray = laser_rays[i];
            const cslibs_math_2d::Point2d   ray_end_point = m_T_l * ray.point;
            const double pz = ray.valid() ? p_hit(gridmap.at(ray_end_point)) + p_rand : 0.0;
            p += pz * pz * pz;
        }
        *it_ps = p;
        p_max = p >= p_max ? p : p_max;
    }

    it_ps = ps_.begin();
    for(auto it = set.begin() ; it != end ; ++it, ++it_ps) {
        *it *= *it_ps / p_max;
    }
}

void LikelihoodFieldModelAMCLNormalized::doSetup(ros::NodeHandle &nh)
{
    auto param_name = [this](const std::string &name){return name_ + "/" + name;};

    max_beams_ = nh.param(param_name("max_beams"), 30);
    z_hit_ = nh.param(param_name("z_hit"), 0.8);
    z_rand_ = nh.param(param_name("z_rand"), 0.05);
    sigma_hit_ = nh.param(param_name("sigma_hit"), 0.15);
    denominator_hit_ = 0.5 * 1.0 / (sigma_hit_ * sigma_hit_);
}
}
