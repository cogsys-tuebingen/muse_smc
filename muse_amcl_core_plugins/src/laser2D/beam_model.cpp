#include "beam_model.h"

using namespace muse_amcl;

BeamModel::BeamModel()
{

}

double BeamModel::apply(Data::ConstPtr &data,
                        Map::ConstPtr &map,
                        ParticleSet::WeightIterator set)
{
    return 0.0;
}

void BeamModel::loadParameters(ros::NodeHandle &nh)
{

}
