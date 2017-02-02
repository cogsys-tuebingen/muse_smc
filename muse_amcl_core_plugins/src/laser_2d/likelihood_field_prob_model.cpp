#include "likelihood_field_prob_model.h"

#include <muse_amcl_core_plugins/laser_2d/laser_scan_2d.hpp>
#include <muse_amcl_core_plugins/maps_2d/distance_gridmap.h>

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(muse_amcl::LikelihoodFieldProbModel, muse_amcl::UpdateModel)

using namespace muse_amcl;

LikelihoodFieldProbModel::LikelihoodFieldProbModel()
{
}

void LikelihoodFieldProbModel::update(const Data::ConstPtr &data,
                                      const Map::ConstPtr &map,
                                      ParticleSet::Weights set)
{
    const maps::DistanceGridMap &gridmap = map->as<maps::DistanceGridMap>();
    const LaserScan2D           &laser_data = data->as<LaserScan2D>();

    /// get laser to base ...

}

void LikelihoodFieldProbModel::doSetup(ros::NodeHandle &nh_private)
{
    max_beams_ = nh_private.param(privateParameter("max_beams"), 30);
    z_hit_ = nh_private.param(privateParameter("z_hit"), 0.8);
    z_rand_ = nh_private.param(privateParameter("z_rand"), 0.05);
    sigma_hit_ = nh_private.param(privateParameter("sigma_hit"), 0.15);
    beam_skip_ = nh_private.param(privateParameter("beam_skip"), true);
    beam_skip_distance_ = nh_private.param(privateParameter("beam_skip_distance"), 0.5);
    beam_skip_threshold_= nh_private.param(privateParameter("beam_skip_threshold"),0.5 );
    beam_skip_error_threshold_= nh_private.param(privateParameter("beam_skip_error"), 0.5);
    /// @todo - fix the default parameters
}
