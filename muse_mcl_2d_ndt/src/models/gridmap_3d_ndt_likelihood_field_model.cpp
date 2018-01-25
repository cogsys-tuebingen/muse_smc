#include <muse_mcl_2d_ndt/models/gridmap_3d_ndt_likelihood_field_model.h>

#include <muse_mcl_2d_stereo/stereo_data.hpp>
#include <muse_mcl_2d_ndt/maps/gridmap_3d.h>

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(muse_mcl_2d_ndt::Gridmap3dNDTLikelihoodFieldModel, muse_mcl_2d::UpdateModel2D)

namespace muse_mcl_2d_ndt {
Gridmap3dNDTLikelihoodFieldModel::Gridmap3dNDTLikelihoodFieldModel()
{
}

void Gridmap3dNDTLikelihoodFieldModel::apply(const data_t::ConstPtr          &data,
                                          const state_space_t::ConstPtr   &map,
                                          sample_set_t::weight_iterator_t set)
{
    if (!map->isType<Gridmap3d>() || !data->isType<muse_mcl_2d_stereo::StereoData>())
        return;

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

    const double bundle_resolution_inv = 1.0 / gridmap.getBundleResolution();
    auto to_bundle_index = [&bundle_resolution_inv](const cslibs_math_3d::Point3d &p) {
        return std::array<int, 3>({{static_cast<int>(std::floor(p(0) * bundle_resolution_inv)),
                                    static_cast<int>(std::floor(p(1) * bundle_resolution_inv)),
                                    static_cast<int>(std::floor(p(2) * bundle_resolution_inv))}});
    };
    auto to_rotation_matrix = [](const double &cos, const double &sin) {
        Eigen::Matrix<double, 3, 3> r(Eigen::Matrix<double, 3, 3>::Identity());
        r(0, 0) = r(1, 1) = cos;
        r(0, 1) = -sin;
        r(1, 0) = sin;
        return r;
    };

    using index_t = std::array<int, 3>;
    using point_t = cslibs_math_3d::Point3d;
    using distribution_t = cslibs_ndt_3d::dynamic_maps::Gridmap::distribution_t;
    using distribution_storage_t = cslibs_ndt_3d::dynamic_maps::Gridmap::distribution_storage_t;

    // local ndt map
    distribution_storage_t local_storage;
    for (const auto &p : *stereo_points) {
        if (p.isNormal()) {
            const index_t &bi = to_bundle_index(p);
            distribution_t *d = local_storage.get(bi);
            (d ? d : &local_storage.insert(bi, distribution_t()))->data().add(p);
        }
    }

    // mixture distribution entries
    for (auto it = set.begin() ; it != set.end() ; ++it) {
        const cslibs_math_2d::Pose2d m_T_s = m_T_w * it.state() * b_T_s; /// stereo camera pose in map coordinates
        const cslibs_math_3d::Pose3d m_T_s_3d(m_T_s.tx(), m_T_s.ty(), m_T_s.yaw());
        double p = 0.0;

        const Eigen::Matrix<double, 3, 3> r = to_rotation_matrix(m_T_s.cos(), m_T_s.sin());
        local_storage.traverse([this, &p, &m_T_s_3d, &gridmap, &r, &to_bundle_index](const index_t& , const distribution_t &d_local) {
            const point_t mean = m_T_s_3d * point_t(d_local.getHandle()->data().getMean());
            const Eigen::Matrix<double, 3, 3> cov = d_local.getHandle()->data().getCovariance();

            const auto &bundle = gridmap.getDistributionBundle(to_bundle_index(mean));
            cslibs_math::statistics::Distribution<3, 3> d;
            for (std::size_t i = 0 ; i < 8 ; ++ i)
                if (bundle->at(i))
                    d += bundle->at(i)->data();

            const Eigen::Matrix<double, 3, 1> mn  = mean.data();
            const Eigen::Matrix<double, 3, 3> inf = (r * cov * r.transpose()).inverse();
            p += d1_ * std::exp(-0.5 * d2_ * double(mn.transpose() * inf * mn));
        });

        *it *= p;
    }
}

void Gridmap3dNDTLikelihoodFieldModel::doSetup(ros::NodeHandle &nh)
{
    auto param_name = [this](const std::string &name){return name_ + "/" + name;};

    max_points_ = nh.param(param_name("max_points"), 30);
    d1_         = nh.param(param_name("d1"), 0.9);
    d2_         = nh.param(param_name("d2"), 0.05);
}
}
