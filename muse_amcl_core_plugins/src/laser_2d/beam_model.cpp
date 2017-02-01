#include "beam_model.h"

#include "laser_scan_2d.hpp"
#include <muse_amcl_core_plugins/maps_2d/binary_gridmap.h>

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(muse_amcl::BeamModel, muse_amcl::UpdateModel)

using namespace muse_amcl;


BeamModel::BeamModel()
{

}

void BeamModel::update(const Data::ConstPtr  &data,
                       const Map::ConstPtr  &map,
                       ParticleSet::Weights set)
{

    const maps::BinaryGridMap *gridmap = map->as<maps::BinaryGridMap>();
    if(!gridmap)
        throw std::runtime_error("[BeamModel]: Trying to use wrong map type with this model.");



}

void BeamModel::doSetup(ros::NodeHandle &nh_private_)
{

}
