#include "beam_model_mle.h"

#include <muse_amcl_core_plugins/laser_2d/laser_scan_2d.hpp>
#include <muse_amcl_core_plugins/maps_2d/binary_gridmap.h>

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(muse_amcl::BeamModelMLE, muse_amcl::UpdateModel)

#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>

using namespace muse_amcl;

BeamModelMLE::BeamModelMLE()
{
}

void BeamModelMLE::update(const Data::ConstPtr  &data,
                       const Map::ConstPtr   &map,
                       ParticleSet::Weights   set)
{
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

//    pcl::PointCloud<pcl::PointXYZ>::Ptr debug(new pcl::PointCloud<pcl::PointXYZ>);
//    debug->header.frame_id = map->getFrame();
//    debug->header.stamp = data->getTimeFrame().end.toNSec() / 1000.0;

    for(auto it = set.begin() ; it != end ; ++it) {
        const math::Pose pose = m_T_w * it.getData().pose_ * b_T_l; /// laser scanner pose in map coordinates
        double p = 1.0;
        for(std::size_t i = 0 ; i < rays_size ;  i+= ray_step) {
            const double        ray_range = laser_rays[i].range_;
            const math::Point   ray_end_point = pose.getPose() * laser_rays[i].point_;
            const double        map_range = gridmap.getRange(pose.getOrigin(), ray_end_point);
            const double pz = probability(ray_range, map_range);
            p += pz * pz * pz;  /// @todo : fix the inprobable thing ;)
//            debug->points.emplace_back(pcl::PointXYZ(ray_end_point.getX(),
//                                                     ray_end_point.getY(),
//                                                     ray_end_point.getZ()));
        }
        *it *= p;
    }

//    pub_debug_.publish(debug);
}

void BeamModelMLE::doSetup(ros::NodeHandle &nh_private)
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

    pub_debug_ = nh_private.advertise<pcl::PointCloud<pcl::PointXYZ>>("/muse_amcl/" + name_, 1);

    Logger &l = Logger::getLogger();
    l.info("max_beams_=" + std::to_string(max_beams_), "UpdateModel:" + name_);
    l.info("z_hit_=" + std::to_string(z_hit_), "UpdateModel:" + name_);
    l.info("z_short_=" + std::to_string(z_short_), "UpdateModel:" + name_);
    l.info("z_max_=" + std::to_string(z_max_), "UpdateModel:" + name_);
    l.info("z_rand_=" + std::to_string(z_rand_), "UpdateModel:" + name_);
    l.info("sigma_hit_=" + std::to_string(sigma_hit_), "UpdateModel:" + name_);
    l.info("lambda_short_=" + std::to_string(lambda_short_), "UpdateModel:" + name_);
    l.info("chi_outlier_=" + std::to_string(chi_outlier_), "UpdateModel:" + name_);
}
