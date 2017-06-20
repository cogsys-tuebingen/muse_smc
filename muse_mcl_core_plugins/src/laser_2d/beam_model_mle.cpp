#include "beam_model_mle.h"

#include <muse_mcl_core_plugins/laser_2d/laser_scan_2d.hpp>
#include <muse_mcl_core_plugins/maps_2d/binary_gridmap.h>

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(muse_mcl::BeamModelMLE, muse_mcl::UpdateModel)


using namespace muse_mcl;

BeamModelMLE::BeamModelMLE()
{
}

void BeamModelMLE::update(const Data::ConstPtr  &data,
                          const Map::ConstPtr   &map,
                          ParticleSet::Weights   set)
{
    if(!map->isType<maps::BinaryGridMap>()) {
        Logger::getLogger().error("The map is of incompatible type!", "UpdateModel:" + name_);
        return;
    }

    if(use_estimated_parameters_)
        parameter_estimator_mle_->getParameters(parameters_);

    const maps::BinaryGridMap &gridmap = map->as<maps::BinaryGridMap>();
    const LaserScan2D         &laser_data = data->as<LaserScan2D>();
    const LaserScan2D::Rays   &laser_rays = laser_data.getRays();

    /// laser to base transform
    tf::Transform b_T_l;
    tf::Transform m_T_w;
    if(!tf_provider_->lookupTransform(robot_base_frame_,
                                      laser_data.getFrame(),
                                      laser_data.getTimeFrame().end,
                                      b_T_l,
                                      tf_timeout_))
        return;
    if(!tf_provider_->lookupTransform(world_frame_,
                                      gridmap.getFrame(),
                                      laser_data.getTimeFrame().end,
                                      m_T_w,
                                      tf_timeout_))
        return;

    const LaserScan2D::Rays rays = laser_data.getRays();
    const ParticleSet::Weights::iterator end = set.end();
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
        const math::Pose pose = m_T_w * it.getData().pose_ * b_T_l; /// laser scanner pose in map coordinates
        const double prior = *it;
        double p = 0.0;
        for(std::size_t i = 0 ; i < rays_size ;  i+= ray_step) {
            const auto &ray = laser_rays[i];
            if(!ray.valid_) {
                p += parameters_.z_max;
            } else {
                const double        ray_range = ray.range_;
                const math::Point   ray_end_point = pose.getPose() * ray.point_;
                const double        map_range = gridmap.getRange(pose.getOrigin(), ray_end_point);
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

void BeamModelMLE::doSetup(ros::NodeHandle &nh_private)
{
    max_beams_                  = nh_private.param(privateParameter("max_beams"), 30);
    parameters_.z_hit           = nh_private.param(privateParameter("z_hit"), 0.8);
    parameters_.z_short         = nh_private.param(privateParameter("z_short"), 0.1);
    parameters_.z_max           = nh_private.param(privateParameter("z_max"), 0.05);
    parameters_.z_rand          = nh_private.param(privateParameter("z_rand"), 0.05);
    parameters_.setSigmaHit(nh_private.param(privateParameter("sigma_hit"), 0.15));
    parameters_.lambda_short    = nh_private.param(privateParameter("lambda_short"), 0.01);
    use_estimated_parameters_   = nh_private.param(privateParameter("use_estimated_parameters"), false);
    use_weights_for_estimation_   = nh_private.param(privateParameter("use_weights_for_estimation"), false);

    std::size_t max_estimation_iterations =
            static_cast<std::size_t>(nh_private.param<int>(privateParameter("max_estimation_iterations"), 20));

    parameter_estimator_mle_.reset(new BeamModelParameterEstimator(parameters_, max_estimation_iterations));

    Logger &l = Logger::getLogger();
    l.info("max_beams_=" + std::to_string(max_beams_), "UpdateModel:" + name_);
    l.info("z_hit_="     + std::to_string(parameters_.z_hit), "UpdateModel:" + name_);
    l.info("z_short_="   + std::to_string(parameters_.z_short), "UpdateModel:" + name_);
    l.info("z_max_="     + std::to_string(parameters_.z_max), "UpdateModel:" + name_);
    l.info("z_rand_="    + std::to_string(parameters_.z_rand), "UpdateModel:" + name_);
    l.info("sigma_hit_=" + std::to_string(parameters_.sigma_hit), "UpdateModel:" + name_);
    l.info("lambda_short_=" + std::to_string(parameters_.lambda_short), "UpdateModel:" + name_);
}
