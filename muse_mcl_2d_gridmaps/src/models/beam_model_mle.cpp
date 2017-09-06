#include "beam_model_mle.h"

#include <muse_mcl_2d_laser/laser/laser_2d_scan.hpp>
#include <muse_mcl_2d_gridmaps/maps//binary_gridmap.h>

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(muse_mcl_2d_gridmaps::BeamModelMLE, muse_mcl_2d::UpdateModel2D)


using namespace muse_mcl_2d_gridmaps;

BeamModelMLE::BeamModelMLE()
{
}

void BeamModelMLE::apply(const data_t::ConstPtr          &data,
                         const state_space_t::ConstPtr   &map,
                         sample_set_t::weight_iterator_t set)
{
    if(!map->isType<maps::BinaryGridMap>()) {
        return;
    }

    if(use_estimated_parameters_)
        parameter_estimator_mle_->getParameters(parameters_);

    const maps::BinaryGridMap &gridmap = map->as<maps::BinaryGridMap>();
    const muse_mcl_2d_laser::LaserScan2D        &laser_data = data->as<muse_mcl_2d_laser::LaserScan2D>();
    const muse_mcl_2d_laser::LaserScan2D::Rays  &laser_rays = laser_data.getRays();


    /// laser to base transform
    muse_mcl_2d::Transform2D b_T_l;
    muse_mcl_2d::Transform2D m_T_w;
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
    const std::size_t ray_step  = std::max(1ul, (rays_size) / max_beams_);
    const double range_max = laser_data.getRangeMax();
    const double p_rand = parameters_.z_rand * 1.0 / range_max;

    /// Mixture distribution
    auto p_hit = [this](const double ray_range, const double map_range) {
        const double dz = ray_range - map_range;
        return parameters_.z_hit * parameters_.denominator_hit *
                std::exp(-dz * dz * parameters_.denominator_exponent_hit);
    };
    auto p_short = [this](const double ray_range, const double map_range) {
        if(ray_range < map_range) {
            return parameters_.z_short *
                    (1.0 / (1.0 - std::exp(-parameters_.lambda_short  * map_range))) *
                    parameters_.lambda_short * exp(-parameters_.lambda_short * ray_range);
        }
        return 0.0;
    };
    auto p_max = [this, range_max](const double ray_range)
    {
        if(ray_range >= range_max)
            return parameters_.z_max * 1.0;
        return 0.0;
    };
    auto p_random = [this, p_rand, range_max](const double ray_range)
    {
        if(ray_range < range_max)
            return p_rand;
        return 0.0;
    };

    auto probability = [p_hit, p_short, p_max, p_random] (const double ray_range, const double map_range)
    {
        return p_hit(ray_range, map_range) + p_short(ray_range, map_range) + p_max(ray_range) + p_random(ray_range);
    };

    std::vector<double> z;
    std::vector<double> z_bar;
    std::vector<double> particle_weights;

    for(auto it = set.begin() ; it != end ; ++it) {
        const muse_mcl_2d::Pose2D m_T_l = m_T_w * it.getData().state * b_T_l; /// laser scanner pose in map coordinates
        const double prior = *it;
        double p = 0.0;
        for(std::size_t i = 0 ; i < rays_size ;  i+= ray_step) {
            const auto &ray = laser_rays[i];
            if(!ray.valid) {
                p += parameters_.z_max;
            } else {
                const double        ray_range = ray.range;
                muse_mcl_2d::Point2D   ray_end_point = m_T_l * ray.point;
                const double        map_range = gridmap.getRange(m_T_l.translation(), ray_end_point);
                const double pz = probability(ray_range, map_range);
                p += std::log(pz);  /// @todo : fix the inprobable thing ;)
                z.emplace_back(ray_range);
                z_bar.emplace_back(map_range);
                particle_weights.emplace_back(prior);
            }
        }
        *it *= std::exp(p);
    }

    if(use_weights_for_estimation_)
        parameter_estimator_mle_->setMeasurements(z, z_bar, particle_weights, range_max);
    else
        parameter_estimator_mle_->setMeasurements(z, z_bar, range_max);
}

void BeamModelMLE::doSetup(ros::NodeHandle &nh)
{
    auto param_name = [this](const std::string &name){return name_ + "/" + name;};

    max_beams_                  = nh.param(param_name("max_beams"), 30);
    parameters_.z_hit           = nh.param(param_name("z_hit"), 0.8);
    parameters_.z_short         = nh.param(param_name("z_short"), 0.1);
    parameters_.z_max           = nh.param(param_name("z_max"), 0.05);
    parameters_.z_rand          = nh.param(param_name("z_rand"), 0.05);
    parameters_.setSigmaHit(nh.param(param_name("sigma_hit"), 0.15));
    parameters_.lambda_short    = nh.param(param_name("lambda_short"), 0.01);
    use_estimated_parameters_   = nh.param(param_name("use_estimated_parameters"), false);
    use_weights_for_estimation_   = nh.param(param_name("use_weights_for_estimation"), false);

    std::size_t max_estimation_iterations =
            static_cast<std::size_t>(nh.param<int>(param_name("max_estimation_iterations"), 20));

    parameter_estimator_mle_.reset(new BeamModelParameterEstimator(parameters_, max_estimation_iterations));
}
