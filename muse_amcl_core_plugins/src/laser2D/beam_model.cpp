#include "beam_model.h"

using namespace muse_amcl;

BeamModel::BeamModel()
{

}

double BeamModel::apply(const Data::ConstPtr &data,
                        const Map::ConstPtr &map,
                        ParticleSet::WeightIterator set)
{
    return 0.0;
}

void BeamModel::doSetup(ros::NodeHandle &nh)
{

}
