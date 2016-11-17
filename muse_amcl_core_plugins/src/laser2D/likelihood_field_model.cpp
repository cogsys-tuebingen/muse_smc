#include "likelihood_field_model.h"

using namespace muse_amcl;

LikelihoodFieldModel::LikelihoodFieldModel()
{
}

double LikelihoodFieldModel::apply(const Data::ConstPtr &data,
                                   const Map::ConstPtr &map,
                                   ParticleSet::WeightIterator set)
{
    return 0.0;
}

void LikelihoodFieldModel::doSetup(ros::NodeHandle &nh)
{

}
