#include <muse_mcl_2d_ndt/models/occupancy_gridmap_3d_ndt_likelihood_field_model_normalized.h>

#include <muse_mcl_2d_stereo/stereo_data.hpp>
#include <muse_mcl_2d_ndt/maps/occupancy_gridmap_3d.h>
#include <muse_mcl_2d_ndt/maps/gridmap_3d.h>

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(muse_mcl_2d_ndt::OccupancyGridmap3dNDTLikelihoodFieldModelNormalized, muse_mcl_2d::UpdateModel2D)

namespace muse_mcl_2d_ndt {
OccupancyGridmap3dNDTLikelihoodFieldModelNormalized::OccupancyGridmap3dNDTLikelihoodFieldModelNormalized()
{
}

void OccupancyGridmap3dNDTLikelihoodFieldModelNormalized::apply(const data_t::ConstPtr          &data,
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
    auto it_ps = ps_.begin();
    double p_max = std::numeric_limits<double>::lowest();
    for (auto it = set.begin() ; it != set.end() ; ++it, ++it_ps) {
        const cslibs_math_2d::Pose2d m_T_s = m_T_w * it.state() * b_T_s; /// stereo camera pose in map coordinates
        const cslibs_math_3d::Pose3d m_T_s_3d(m_T_s.tx(), m_T_s.ty(), m_T_s.yaw());
        double p = 0.0;

        const Eigen::Matrix<double, 3, 3> r = to_rotation_matrix(m_T_s.cos(), m_T_s.sin());
        local_storage.traverse([this, &p, &m_T_s_3d, &gridmap, &r, &to_bundle_index](const index_t& , const distribution_t &d_local) {
            const point_t mean = m_T_s_3d * point_t(d_local.getHandle()->data().getMean());
            const Eigen::Matrix<double, 3, 3> cov = d_local.getHandle()->data().getCovariance();

            const auto &bundle = gridmap.getDistributionBundle(to_bundle_index(mean));
            cslibs_math::statistics::Distribution<3, 3> d;
            double occupancy = 0.0;
            for (std::size_t i = 0 ; i < 8 ; ++ i) {
                if (bundle->at(i)) {
                    if (bundle->at(i)->getDistribution())
                        d += *(bundle->at(i)->getDistribution());
                    occupancy += 0.125 * bundle->at(i)->getOccupancy(inverse_model_);
                }
            }

            const Eigen::Matrix<double, 3, 1> mn  = mean.data();
            const Eigen::Matrix<double, 3, 3> inf = (r * cov * r.transpose()).inverse();
            p += occupancy * (d1_ * std::exp(-0.5 * d2_ * double(mn.transpose() * inf * mn)));
        });

        *it_ps = p;
        p_max = p >= p_max ? p : p_max;
    }

    it_ps = ps_.begin();
    for (auto it = set.begin() ; it != set.end() ; ++it, ++it_ps)
        *it *= *it_ps / p_max;
}

void OccupancyGridmap3dNDTLikelihoodFieldModelNormalized::doSetup(ros::NodeHandle &nh)
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
