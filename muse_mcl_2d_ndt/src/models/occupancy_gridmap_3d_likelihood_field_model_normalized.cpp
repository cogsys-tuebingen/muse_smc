#include <muse_mcl_2d_ndt/models/occupancy_gridmap_3d_likelihood_field_model_normalized.h>

#include <muse_mcl_2d_stereo/stereo_data.hpp>
#include <muse_mcl_2d_ndt/maps/occupancy_gridmap_3d.h>

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(muse_mcl_2d_ndt::OccupancyGridmap3dLikelihoodFieldModelNormalized, muse_mcl_2d::UpdateModel2D)

namespace muse_mcl_2d_ndt {
OccupancyGridmap3dLikelihoodFieldModelNormalized::OccupancyGridmap3dLikelihoodFieldModelNormalized()
{
}

void OccupancyGridmap3dLikelihoodFieldModelNormalized::apply(const data_t::ConstPtr          &data,
                                          const state_space_t::ConstPtr   &map,
                                          sample_set_t::weight_iterator_t set)
{
    if (!map->isType<OccupancyGridmap3d>() || !data->isType<muse_mcl_2d_stereo::StereoData>())
        return;
    if (ps_.size() != set.capacity())
        ps_.resize(set.capacity());
    std::fill(ps_.begin(), ps_.end(), 0.0);

    const cslibs_ndt_3d::dynamic_maps::OccupancyGridmap &gridmap       = *(map->as<OccupancyGridmap3d>().data());
    const muse_mcl_2d_stereo::StereoData                &stereo_data   = data->as<muse_mcl_2d_stereo::StereoData>();
    const cslibs_math_3d::Pointcloud3d::Ptr             &stereo_points = stereo_data.getPoints();

    /// stereo to base transform
    cslibs_math_2d::Transform2d b_T_s;
    cslibs_math_2d::Transform2d m_T_w;
    if (!tf_->lookupTransform(robot_base_frame_,
                              stereo_data.getFrame(),
                              ros::Time(stereo_data.getTimeFrame().end.seconds()),
                              b_T_s,
                              tf_timeout_))
        return;
    if (!tf_->lookupTransform(world_frame_,
                              map->getFrame(),
                              ros::Time(stereo_data.getTimeFrame().end.seconds()),
                              m_T_w,
                              tf_timeout_))
        return;

    const std::size_t points_size = stereo_points->size();
    const std::size_t points_step = std::max(1ul, points_size / max_points_);

    // mixture distribution entries
    const double bundle_resolution_inv = 1.0 / gridmap.getBundleResolution();
    auto to_bundle_index = [&bundle_resolution_inv](const cslibs_math_3d::Point3d &p) {
        return std::array<int, 3>({{static_cast<int>(std::floor(p(0) * bundle_resolution_inv)),
                                    static_cast<int>(std::floor(p(1) * bundle_resolution_inv)),
                                    static_cast<int>(std::floor(p(2) * bundle_resolution_inv))}});
    };
    auto likelihood = [this](const cslibs_math_3d::Point3d &p, const cslibs_math::statistics::Distribution<3, 3>::Ptr &d) {
        if (!d) return 0.0;
        const auto &q         = p.data() - d->getMean();
        const double exponent = -0.5 * d2_ * double(q.transpose() * d->getInformationMatrix() * q);
        return d1_ * std::exp(exponent);
    };
    auto occupancy_likelihood = [this, &likelihood](const cslibs_math_3d::Point3d &p, const cslibs_ndt::OccupancyDistribution<3>* d) {
        return d ? d->getOccupancy(inverse_model_) * likelihood(p, d->getDistribution()) : 0.0;
    };
    auto bundle_likelihood = [&gridmap, &to_bundle_index, &occupancy_likelihood](const cslibs_math_3d::Point3d &p) {
        const auto &bundle = gridmap.getDistributionBundle(to_bundle_index(p));
        return 0.125 * (occupancy_likelihood(p, bundle->at(0)) +
                        occupancy_likelihood(p, bundle->at(1)) +
                        occupancy_likelihood(p, bundle->at(2)) +
                        occupancy_likelihood(p, bundle->at(3)) +
                        occupancy_likelihood(p, bundle->at(4)) +
                        occupancy_likelihood(p, bundle->at(5)) +
                        occupancy_likelihood(p, bundle->at(6)) +
                        occupancy_likelihood(p, bundle->at(7)));
    };

    auto it_ps = ps_.begin();
    double p_max = std::numeric_limits<double>::lowest();
    for (auto it = set.begin() ; it != set.end() ; ++it, ++it_ps) {
        const cslibs_math_2d::Pose2d m_T_s = m_T_w * it.state() * b_T_s; /// stereo camera pose in map coordinates
        const cslibs_math_3d::Pose3d m_T_s_3d(m_T_s.tx(), m_T_s.ty(), m_T_s.yaw());
        double p = 0.0;
        for (std::size_t i = 0 ; i < points_size ;  i+= points_step) {
            const auto &point = stereo_points->at(i);
            const cslibs_math_3d::Point3d map_point = m_T_s_3d * point;
            p += map_point.isNormal() ? bundle_likelihood(map_point) : 0;
        }
        *it_ps = p;
        p_max = p >= p_max ? p : p_max;
    }

    it_ps = ps_.begin();
    for (auto it = set.begin() ; it != set.end() ; ++it, ++it_ps)
        *it *= *it_ps / p_max;
}

void OccupancyGridmap3dLikelihoodFieldModelNormalized::doSetup(ros::NodeHandle &nh)
{
    auto param_name = [this](const std::string &name){return name_ + "/" + name;};

    max_points_ = nh.param(param_name("max_points"), 30);
    d1_         = nh.param(param_name("d1"), 0.9);
    d2_         = nh.param(param_name("d2"), 0.05);

    occupied_threshold_         = nh.param(param_name("occupied_threshold"), 0.196);
    const double prob_prior     = nh.param(param_name("prob_prior"), 0.5);
    const double prob_free      = nh.param(param_name("prob_free"), 0.45);
    const double prob_occupied  = nh.param(param_name("prob_occupied"), 0.65);
    inverse_model_.reset(new cslibs_gridmaps::utility::InverseModel(prob_prior, prob_free, prob_occupied));
}
}
