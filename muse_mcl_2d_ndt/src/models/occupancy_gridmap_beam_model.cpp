#include <muse_mcl_2d_ndt/models/occupancy_gridmap_beam_model.h>

#include <muse_mcl_2d_laser/laserscan_2d.hpp>
#include <muse_mcl_2d_ndt/maps/occupancy_gridmap_2d.h>

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(muse_mcl_2d_ndt::OccupancyGridmapBeamModel, muse_mcl_2d::UpdateModel2D)

namespace muse_mcl_2d_ndt {
OccupancyGridmapBeamModel::OccupancyGridmapBeamModel()
{
}

void OccupancyGridmapBeamModel::apply(const data_t::ConstPtr          &data,
                                      const state_space_t::ConstPtr   &map,
                                      sample_set_t::weight_iterator_t set)
{
    if (!map->isType<OccupancyGridmap2d>() || !data->isType<muse_mcl_2d_laser::LaserScan2D>() || !inverse_model_)
        return;

    const cslibs_ndt_2d::dynamic_maps::OccupancyGridmap &gridmap    = *(map->as<OccupancyGridmap2d>().data());
    const muse_mcl_2d_laser::LaserScan2D                &laser_data = data->as<muse_mcl_2d_laser::LaserScan2D>();
    const muse_mcl_2d_laser::LaserScan2D::rays_t        &laser_rays = laser_data.getRays();

    /// laser to base transform
    cslibs_math_2d::Transform2d b_T_l;
    cslibs_math_2d::Transform2d m_T_w;
    if (!tf_->lookupTransform(robot_base_frame_,
                              laser_data.getFrame(),
                              ros::Time(laser_data.getTimeFrame().end.seconds()),
                              b_T_l,
                              tf_timeout_))
        return;
    if (!tf_->lookupTransform(world_frame_,
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
    auto pow2 = [](const double x) {return x*x;};
    const double bundle_resolution_inv = 1.0 / gridmap.getBundleResolution();
    auto to_bundle_index = [&bundle_resolution_inv](const cslibs_math_2d::Vector2d &p) {
        return std::array<int, 2>({{static_cast<int>(std::floor(p(0) * bundle_resolution_inv)),
                                    static_cast<int>(std::floor(p(1) * bundle_resolution_inv))}});
    };
    auto p_hit = [this, &gridmap, &to_bundle_index, &pow2](
            const cslibs_math_2d::Vector2d &ray_end_point, const double &ray_range, const double &map_range) {
        return z_hit_ * denominator_hit_ * std::exp(pow2(ray_range - map_range) * denominator_exponent_hit_)
                * gridmap.sampleNonNormalized(ray_end_point,
                                              to_bundle_index(ray_end_point),
                                              inverse_model_);
    };
    auto p_short = [this](const double &ray_range, const double &map_range) {
        return ray_range < map_range ? z_short_ * (1.0 / (1.0 - std::exp(-lambda_short_  * map_range))) * lambda_short_ * std::exp(-lambda_short_ * ray_range)
                                       : 0.0;
    };
    auto p_max = [this, range_max](const double &ray_range) {
        return ray_range >= range_max ? z_max_ : 0.0;
    };
    auto p_random = [this, range_max, p_rand](const double &ray_range) {
        return ray_range < range_max ? p_rand : 0.0;
    };
    auto probability = [this, &gridmap, &p_hit, &p_short, &p_max, &p_random]
            (const muse_mcl_2d_laser::LaserScan2D::Ray &ray, const cslibs_math_2d::Pose2d &m_T_l) {
        const double ray_range = ray.range;
        auto         ray_end_point = m_T_l * ray.point;
        const double map_range = gridmap.getRange(m_T_l.translation(), ray_end_point, inverse_model_, occupied_threshold_);
        return p_hit(ray_end_point, ray_range, map_range) + p_short(ray_range, map_range) + p_max(ray_range) + p_random(ray_range);
    };

    for (auto it = set.begin() ; it != end ; ++it) {
        const cslibs_math_2d::Pose2d m_T_l = m_T_w * it.state() * b_T_l; /// laser scanner pose in map coordinates
        double p = 1.0;
        for (std::size_t i = 0 ; i < rays_size ;  i+= ray_step) {
            const auto &ray = laser_rays[i];
            p *= ray.valid() ? probability(ray, m_T_l) : z_max_;
        }
        *it *= p;
    }
}

void OccupancyGridmapBeamModel::doSetup(ros::NodeHandle &nh)
{
    auto param_name = [this](const std::string &name){return name_ + "/" + name;};

    max_beams_                  = nh.param(param_name("max_beams"), 30);
    z_hit_                      = nh.param(param_name("z_hit"), 0.8);
    z_short_                    = nh.param(param_name("z_short"), 0.1);
    z_max_                      = nh.param(param_name("z_max"), 0.05);
    z_rand_                     = nh.param(param_name("z_rand"), 0.05);
    sigma_hit_                  = nh.param(param_name("sigma_hit"), 0.15);
    denominator_exponent_hit_   = -0.5 * 1.0 / (sigma_hit_ * sigma_hit_);
    denominator_hit_            = 1.0 / sqrt(2.0 * M_PI * sigma_hit_ * sigma_hit_);
    lambda_short_               = nh.param(param_name("lambda_short"), 0.01);
    occupied_threshold_         = nh.param(param_name("occupied_threshold"), 0.196);

    const double prob_prior     = nh.param(param_name("prob_prior"), 0.5);
    const double prob_free      = nh.param(param_name("prob_free"), 0.45);
    const double prob_occupied  = nh.param(param_name("prob_occupied"), 0.65);
    inverse_model_.reset(new cslibs_gridmaps::utility::InverseModel(prob_prior, prob_free, prob_occupied));
}
}
