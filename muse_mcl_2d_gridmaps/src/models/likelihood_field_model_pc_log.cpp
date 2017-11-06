#include "likelihood_field_model_pc_log.h"

#include <muse_mcl_2d_laser/laserscan_2d.hpp>

#include <muse_mcl_2d_gridmaps/maps/likelihood_field_gridmap.h>

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(muse_mcl_2d_gridmaps::LikelihoodFieldModelPCLog, muse_mcl_2d::UpdateModel2D)

using namespace muse_mcl_2d_gridmaps;


LikelihoodFieldModelPCLog::LikelihoodFieldModelPCLog()
{
}

void LikelihoodFieldModelPCLog::apply(const data_t::ConstPtr       &data,
                                      const state_space_t::ConstPtr   &map,
                                      sample_set_t::weight_iterator_t set)
{
    if(!map->isType<LikelihoodFieldGridmap>()) {
        return;
    }

    if(ps_.size() != set.capacity()) {
        ps_.resize(set.capacity(), 0.0);
    }

    const cslibs_gridmaps::static_maps::LikelihoodFieldGridmap  &gridmap      = *(map->as<LikelihoodFieldGridmap>().data());
    const muse_mcl_2d_laser::LaserScan2D                        &laser_data   = data->as<muse_mcl_2d_laser::LaserScan2D>();
    const muse_mcl_2d_laser::LaserScan2D::rays_t                &laser_rays   = laser_data.getRays();

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
    const double range_max      = laser_data.getLinearMax();
    const double p_rand         = z_rand_ * 1.0 / range_max;

    for(auto it = set.begin() ; it != end ; ++it) {
        const cslibs_math_2d::Pose2d m_T_l = m_T_w * it.state() * b_T_l; /// laser scanner pose in map coordinates
        double p = 1.0;
        for(std::size_t i = 0 ; i < rays_size ;  i+= ray_step) {
            const auto &ray = laser_rays[i];
            const cslibs_math_2d::Point2d ray_end_point = m_T_l * ray.point;
            const double pz = ray.valid() ? z_hit_ * gridmap.at(ray_end_point) + p_rand : 1.0;
            p *= pz;
        }
        *it *= p;
    }

    auto it_ps = ps_.begin();
    double log_max = std::numeric_limits<double>::lowest();
    for(auto it = set.const_begin() ; it != const_end ; ++it, ++it_ps) {
        const cslibs_math_2d::Pose2d m_T_l = m_T_w * it->state * b_T_l; /// laser scanner pose in map coordinates
        double p = 0.0;
        for(std::size_t i = 0 ; i < rays_size ;  i+= ray_step) {
            const auto &ray = laser_rays[i];
            const cslibs_math_2d::Point2d ray_end_point = m_T_l * ray.point;
            const double pz = ray.valid() ? z_hit_ * gridmap.at(ray_end_point) + p_rand : 1.0;
            p += std::log(pz);
        }
        *it_ps = p;
        log_max = p >= log_max ? p : log_max;
    }

    it_ps = ps_.begin();
    for(auto it = set.begin() ; it != end ; ++it, ++it_ps) {
        *it *= std::exp(*it_ps - log_max);
    }
}

void LikelihoodFieldModelPCLog::doSetup(ros::NodeHandle &nh)
{
    auto param_name = [this](const std::string &name){return name_ + "/" + name;};

    max_beams_                = nh.param(param_name("max_beams"), 30);
    z_hit_                    = nh.param(param_name("z_hit"), 0.8);
    z_rand_                   = nh.param(param_name("z_rand"), 0.2);
}
