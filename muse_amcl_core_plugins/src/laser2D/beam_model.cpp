#include "beam_model.h"

using namespace muse_amcl;

BeamModel::BeamModel()
{

}

double BeamModel::apply(Data::ConstPtr &data,
                        ParticleSet::WeightIterator set)
{
    return 0.0;
}

void BeamModel::setMap(Map::ConstPtr &map)
{

}

bool BeamModel::hasMap() const
{
    return true;
}

void BeamModel::loadParameters(ros::NodeHandle &nh)
{

}
