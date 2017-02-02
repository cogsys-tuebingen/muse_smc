#include "likelihood_field_prob_model.h"

#include <muse_amcl_core_plugins/maps_2d/binary_gridmap.h>

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
    const maps::BinaryGridMap *gridmap = map->as<maps::BinaryGridMap>();
    if(!gridmap)
        throw std::runtime_error("[LikelihoodFieldProb]: Trying to use wrong map type with this model.");


}

void LikelihoodFieldProbModel::doSetup(ros::NodeHandle &nh)
{

}
