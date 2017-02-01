#include "likelihood_field_prob_model.h"

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
}

void LikelihoodFieldProbModel::doSetup(ros::NodeHandle &nh)
{

}
