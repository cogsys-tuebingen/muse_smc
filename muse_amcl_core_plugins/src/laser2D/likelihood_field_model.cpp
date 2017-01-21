#include "likelihood_field_model.h"

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(muse_amcl::LikelihoodFieldModel, muse_amcl::Update)

using namespace muse_amcl;


LikelihoodFieldModel::LikelihoodFieldModel()
{
}

double LikelihoodFieldModel::apply(const Data::ConstPtr &data,
                                   const Map::ConstPtr &map,
                                   ParticleSet::Weights set)
{
    return 0.0;
}

void LikelihoodFieldModel::doSetup(ros::NodeHandle &nh)
{

}
