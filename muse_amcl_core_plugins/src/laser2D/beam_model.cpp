#include "beam_model.h"

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
}

void BeamModel::doSetup(ros::NodeHandle &nh)
{

}
