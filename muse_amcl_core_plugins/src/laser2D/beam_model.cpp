#include "beam_model.h"

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(muse_amcl::BeamModel, muse_amcl::Update)

using namespace muse_amcl;


BeamModel::BeamModel()
{

}

double BeamModel::apply(const Data::ConstPtr &data,
                        const Map::ConstPtr &map,
                        ParticleSet::Weights set)
{
    return 0.0;
}

void BeamModel::doSetup(ros::NodeHandle &nh)
{

}
