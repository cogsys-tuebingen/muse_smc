#include <muse_mcl_2d_ndt/models/occupancy_gridmap_2d_likelihood_field_model.h>

#include <muse_mcl_2d_laser/laserscan_2d.hpp>
#include <muse_mcl_2d_ndt/maps/occupancy_gridmap_2d.h>

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(muse_mcl_2d_ndt::OccupancyGridmap2dLikelihoodFieldModel, muse_mcl_2d::UpdateModel2D)

namespace muse_mcl_2d_ndt {
OccupancyGridmap2dLikelihoodFieldModel::OccupancyGridmap2dLikelihoodFieldModel()
{
}

void OccupancyGridmap2dLikelihoodFieldModel::apply(const data_t::ConstPtr &data,
                                      const state_space_t::ConstPtr       &map,
                                      sample_set_t::weight_iterator_t     set)
{
    if (!map->isType<OccupancyGridmap2d>() || !data->isType<muse_mcl_2d_laser::LaserScan2D>() || !inverse_model_)
        return;

    const cslibs_ndt_2d::dynamic_maps::OccupancyGridmap &gridmap    = *(map->as<OccupancyGridmap2d>().data());
    const muse_mcl_2d_laser::LaserScan2D                &laser_data = data->as<muse_mcl_2d_laser::LaserScan2D>();
    const muse_mcl_2d_laser::LaserScan2D::rays_t        &laser_rays = laser_data.getRays();

    /// laser to base transform
    cslibs_math_2d::Transform2d b_T_l, m_T_w;
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
    const std::size_t rays_size = rays.size();
    const std::size_t ray_step  = std::max(1ul, rays_size / max_points_);

    /// mixture distribution entries
    const double bundle_resolution_inv = 1.0 / gridmap.getBundleResolution();
    auto to_bundle_index = [&bundle_resolution_inv](const cslibs_math_2d::Vector2d &p) {
        return std::array<int, 2>({{static_cast<int>(std::floor(p(0) * bundle_resolution_inv)),
                                    static_cast<int>(std::floor(p(1) * bundle_resolution_inv))}});
    };
    auto likelihood = [this](const cslibs_math_2d::Point2d &p,
                             const cslibs_math::statistics::Distribution<2, 3>::Ptr &d,
                             const double &inv_occ) {
        auto apply = [&p, &d, &inv_occ, this](){
            const auto &q         = p.data() - d->getMean();
            const double exponent = -0.5 * d2_ * inv_occ * double(q.transpose() * d->getInformationMatrix() * q);
            const double e = d1_ * std::exp(exponent);
            return std::isnormal(e) ? e : 0.0;
        };
        return !d ? 0.0 : apply();
    };
    auto occupancy_likelihood = [this, &likelihood](const cslibs_math_2d::Point2d &p,
                                                    const cslibs_ndt::OccupancyDistribution<2>* d) {
        double occ = d ? d->getOccupancy(inverse_model_) : 0.0;
        double ndt = d ? likelihood(p, d->getDistribution(), 1.0 - occ) : 0.0;

        return ndt;
    };
    auto bundle_likelihood = [this, &gridmap, &to_bundle_index, &occupancy_likelihood](const cslibs_math_2d::Point2d &p) {
        const auto &bundle = gridmap.getDistributionBundle(to_bundle_index(p));
        return 0.25 * (occupancy_likelihood(p, bundle->at(0)) +
                       occupancy_likelihood(p, bundle->at(1)) +
                       occupancy_likelihood(p, bundle->at(2)) +
                       occupancy_likelihood(p, bundle->at(3)));
    };

    auto pow3 = [](const double& x) {
        return x*x*x;
    };
    for (auto it = set.begin() ; it != set.end() ; ++it) {
        const cslibs_math_2d::Pose2d m_T_l = m_T_w * it.state() * b_T_l; /// laser scanner pose in map coordinates
        double p = 1.0;
        for (std::size_t i = 0 ; i < rays_size ;  i+= ray_step) {
            const auto &ray = laser_rays[i];            
            const cslibs_math_2d::Point2d map_point = m_T_l * ray.point;
            p *= ray.valid() && map_point.isNormal() ? pow3(bundle_likelihood(map_point)) : 0.0;
        }
        *it *= p;
    }
}

void OccupancyGridmap2dLikelihoodFieldModel::doSetup(ros::NodeHandle &nh)
{
    auto param_name = [this](const std::string &name){return name_ + "/" + name;};

    max_points_ = nh.param(param_name("max_points"), 100);
    d1_         = nh.param(param_name("d1"), 0.95);
    d2_         = nh.param(param_name("d2"), 0.05);

    const double prob_prior     = nh.param(param_name("prob_prior"), 0.5);
    const double prob_free      = nh.param(param_name("prob_free"), 0.45);
    const double prob_occupied  = nh.param(param_name("prob_occupied"), 0.65);
    inverse_model_.reset(new cslibs_gridmaps::utility::InverseModel(prob_prior, prob_free, prob_occupied));
}
}
