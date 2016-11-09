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

void LikelihoodFieldModel::setMap(Map::ConstPtr &map)
{

}

bool LikelihoodFieldModel::hasMap() const
{
    return true;
}

void LikelihoodFieldModel::loadParameters(ros::NodeHandle &nh)
{

}
