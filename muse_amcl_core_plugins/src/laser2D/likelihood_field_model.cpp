#include "likelihood_field_model.h"

using namespace muse_amcl;

LikelihoodFieldModel::LikelihoodFieldModel()
{
}

double LikelihoodFieldModel::apply(Data::ConstPtr &data,
                                   ParticleSet::WeightIterator set)
{
    return 0.0;
}

void LikelihoodFieldModel::loadParameters(ros::NodeHandle &nh)
{

}
