#include "likelihood_field_model.h"

#include <muse_amcl_core_plugins/maps_2d/binary_gridmap.h>

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(muse_amcl::LikelihoodFieldModel, muse_amcl::UpdateModel)

using namespace muse_amcl;


LikelihoodFieldModel::LikelihoodFieldModel()
{
}

void LikelihoodFieldModel::update(const Data::ConstPtr &data,
                                  const Map::ConstPtr  &map,
                                  ParticleSet::Weights  set)
{
    const maps::BinaryGridMap *gridmap = map->as<maps::BinaryGridMap>();
    if(!gridmap)
        throw std::runtime_error("[LikelihoodField]: Trying to use wrong map type with this model.");

}

void LikelihoodFieldModel::doSetup(ros::NodeHandle &nh_private)
{
    max_beams_ = nh_private.param(privateParameter("max_beams"), 30);
    z_hit_ = nh_private.param(privateParameter("z_hit"), 0.8);
    z_rand_ = nh_private.param(privateParameter("z_rand"), 0.05);
    sigma_hit_ = nh_private.param(privateParameter("sigma_hit"), 0.15);
}
