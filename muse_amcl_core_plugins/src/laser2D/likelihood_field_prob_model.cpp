#include "likelihood_field_prob_model.h"

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(muse_amcl::LikelihoodFieldProbModel, muse_amcl::Update)

using namespace muse_amcl;

LikelihoodFieldProbModel::LikelihoodFieldProbModel()
{

}

double LikelihoodFieldProbModel::apply(Data::ConstPtr &data,
                                       ParticleSet::WeightIterator set)
{
    return 0.0;
}

void LikelihoodFieldProbModel::setMap(Map::ConstPtr &map)
{

}

bool LikelihoodFieldProbModel::hasMap() const
{
    return true;
}

void LikelihoodFieldProbModel::loadParameters(ros::NodeHandle &nh)
{

}
