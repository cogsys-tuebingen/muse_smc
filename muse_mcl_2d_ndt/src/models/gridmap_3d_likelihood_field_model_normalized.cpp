#include <muse_mcl_2d_ndt/models/gridmap_3d_likelihood_field_model_normalized.h>

#include <muse_mcl_2d_stereo/stereo_data.hpp>
#include <muse_mcl_2d_ndt/maps/gridmap_3d.h>

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(muse_mcl_2d_ndt::Gridmap3dLikelihoodFieldModelNormalized, muse_mcl_2d::UpdateModel2D)

namespace muse_mcl_2d_ndt {
Gridmap3dLikelihoodFieldModelNormalized::Gridmap3dLikelihoodFieldModelNormalized()
{
}

void Gridmap3dLikelihoodFieldModelNormalized::apply(const data_t::ConstPtr          &data,
                                          const state_space_t::ConstPtr   &map,
                                          sample_set_t::weight_iterator_t set)
{
    if (!map->isType<Gridmap3d>() || !data->isType<muse_mcl_2d_stereo::StereoData>())
        return;
    if (ps_.size() != set.capacity())
        ps_.resize(set.capacity());
    std::fill(ps_.begin(), ps_.end(), 0.0);

    const cslibs_ndt_3d::dynamic_maps::Gridmap &gridmap       = *(map->as<Gridmap3d>().data());
    const muse_mcl_2d_stereo::StereoData       &stereo_data   = data->as<muse_mcl_2d_stereo::StereoData>();
    const cslibs_math_3d::Pointcloud3d::Ptr    &stereo_points = stereo_data.getPoints();

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

    const std::size_t points_size = stereo_points.size();
    const std::size_t points_step = std::max(1ul, stereo_points / max_points_);

    // mixture distribution entries
    const double bundle_resolution_inv = 1.0 / gridmap.getBundleResolution();
    auto to_bundle_index = [&bundle_resolution_inv](const cslibs_math_3d::Point3d &p) {
        return std::array<int, 3>({{static_cast<int>(std::floor(p(0) * bundle_resolution_inv)),
                                    static_cast<int>(std::floor(p(1) * bundle_resolution_inv)),
                                    static_cast<int>(std::floor(p(2) * bundle_resolution_inv))}});
    };
    auto likelihood = [this](const cslibs_math_3d::Point3d &p, const cslibs_math::statistics::Distribution<3, 3> &d) {
        const auto &q         = p - d.getMean();
        const double exponent = -0.5 * d2_ * double(q.transpose() * d.getInformationMatrix() * q);
        return d1_ * std::exp(exponent);
    };
    auto bundle_likelihood = [&gridmap, &to_bundle_index, &likelihood](const cslibs_math_3d::Point3d &p) {
        const auto &bundle = gridmap.getDistributionBundle(to_bundle_index(p));
        return likelihood(p, bundle->at(0)->getHandle()->data()) +
               likelihood(p, bundle->at(1)->getHandle()->data()) +
               likelihood(p, bundle->at(2)->getHandle()->data()) +
               likelihood(p, bundle->at(3)->getHandle()->data()) +
               likelihood(p, bundle->at(4)->getHandle()->data()) +
               likelihood(p, bundle->at(5)->getHandle()->data()) +
               likelihood(p, bundle->at(6)->getHandle()->data()) +
               likelihood(p, bundle->at(7)->getHandle()->data());
    };

    auto it_ps = ps_.begin();
    double p_max = std::numeric_limits<double>::lowest();
    for (auto it = set.const_begin() ; it != set.const_end() ; ++it, ++it_ps) {
        const cslibs_math_2d::Pose2d m_T_s = m_T_w * it.state() * b_T_s; /// stereo camera pose in map coordinates
        double p = 1.0;
        for (std::size_t i = 0 ; i < points_size ;  i+= points_step) {
            const auto &point = stereo_points[i];
            const cslibs_math_3d::Point3d map_point = m_T_s * point;
            p += map_point.isNormal() ? bundle_likelihood(map_point) : 0;
        }
        *it_ps = p;
        p_max = p >= p_max ? p : p_max;
    }

    it_ps = ps_.begin();
    for(auto it = set.begin() ; it != set.end() ; ++it, ++it_ps)
        *it *= *it_ps / p_max;
}

void Gridmap3dLikelihoodFieldModelNormalized::doSetup(ros::NodeHandle &nh)
{
    auto param_name = [this](const std::string &name){return name_ + "/" + name;};

    max_points_ = nh.param(param_name("max_points"), 30);
    d1_         = nh.param(param_name("d1"), 0.9);
    d2_         = nh.param(param_name("d1"), 0.05);
}
}
