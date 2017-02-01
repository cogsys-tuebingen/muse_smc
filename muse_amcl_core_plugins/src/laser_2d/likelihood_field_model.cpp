#include "likelihood_field_model.h"

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(muse_amcl::LikelihoodFieldModel, muse_amcl::UpdateModel)

using namespace muse_amcl;


LikelihoodFieldModel::LikelihoodFieldModel()
{
}

void LikelihoodFieldModel::update(const Data::ConstPtr &data,
                                  const Map::ConstPtr &map,
                                  ParticleSet::Weights set)
{

}

void LikelihoodFieldModel::doSetup(ros::NodeHandle &nh)
{

}
