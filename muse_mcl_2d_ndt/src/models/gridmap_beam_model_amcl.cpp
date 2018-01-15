#include <muse_mcl_2d_ndt/models/gridmap_beam_model_amcl.h>

#include <muse_mcl_2d_laser/laserscan_2d.hpp>
#include <muse_mcl_2d_ndt/maps/gridmap.h>

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(muse_mcl_2d_ndt::GridmapBeamModelAMCL, muse_mcl_2d::UpdateModel2D)

namespace muse_mcl_2d_ndt {
GridmapBeamModelAMCL::GridmapBeamModelAMCL()
{
}

void GridmapBeamModelAMCL::apply(const data_t::ConstPtr          &data,
                                 const state_space_t::ConstPtr   &map,
                                 sample_set_t::weight_iterator_t set)
{
    if(!map->isType<Gridmap>() || !data->isType<muse_mcl_2d_laser::LaserScan2D>())
        return;

    const cslibs_ndt_2d::dynamic_maps::Gridmap   &gridmap    = *(map->as<Gridmap>().data());
    const muse_mcl_2d_laser::LaserScan2D         &laser_data = data->as<muse_mcl_2d_laser::LaserScan2D>();
    const muse_mcl_2d_laser::LaserScan2D::rays_t &laser_rays = laser_data.getRays();

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
    const std::size_t rays_size = rays.size();
    const std::size_t ray_step  = std::max(1ul, rays_size / max_beams_);
    const double range_max      = laser_data.getLinearMax();
    const double p_rand         = z_rand_ * 1.0 / range_max;


    /// mixture distribution entries
    auto pow3 = [](const double x) {return x*x*x;};
    const double bundle_resolution_inv = 1.0 / gridmap.getBundleResolution();
    auto to_bundle_index = [&bundle_resolution_inv](const cslibs_math_2d::Vector2d &p) {
        return std::array<int, 2>({{static_cast<int>(std::floor(p(0) * bundle_resolution_inv)),
                                    static_cast<int>(std::floor(p(1) * bundle_resolution_inv))}});
    };
    auto p_hit = [this, &gridmap, &to_bundle_index](const cslibs_math_2d::Vector2d &ray_end_point) {
        return z_hit_ * gridmap.sampleNonNormalized(ray_end_point,
                                                    to_bundle_index(ray_end_point));
    };

    for(auto it = set.begin() ; it != end ; ++it) {
        const cslibs_math_2d::Pose2d m_T_l = m_T_w * it.state() * b_T_l; /// laser scanner pose in map coordinates
        double p = 1.0;
        for(std::size_t i = 0 ; i < rays_size ;  i+= ray_step) {
            const auto &ray = laser_rays[i];
            const cslibs_math_2d::Point2d ray_end_point = m_T_l * ray.point;
            p += ray.valid() ? pow3(p_hit(ray_end_point) + p_rand) : pow3(z_max_);
        }
        *it *= p;
    }
}

void GridmapBeamModelAMCL::doSetup(ros::NodeHandle &nh)
{
    auto param_name = [this](const std::string &name){return name_ + "/" + name;};

    max_beams_                  = nh.param(param_name("max_beams"), 30);
    z_hit_                      = nh.param(param_name("z_hit"), 0.8);
    z_short_                    = nh.param(param_name("z_short"), 0.1);
    z_max_                      = nh.param(param_name("z_max"), 0.05);
    z_rand_                     = nh.param(param_name("z_rand"), 0.05);
    sigma_hit_                  = nh.param(param_name("sigma_hit"), 0.15);
    denominator_exponent_hit_   = -0.5 * 1.0 / (sigma_hit_ * sigma_hit_);
    lambda_short_               = nh.param(param_name("lambda_short"), 0.01);
}
}
