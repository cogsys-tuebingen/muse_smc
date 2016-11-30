#include "likelihood_field_prob_model.h"

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(muse_amcl::LikelihoodFieldProbModel, muse_amcl::Update)

using namespace muse_amcl;


LikelihoodFieldProbModel::LikelihoodFieldProbModel()
{

}

double LikelihoodFieldProbModel::apply(const Data::ConstPtr &data,
                                       const Map::ConstPtr &map,
                                       ParticleSet::WeightIterator set)
{
    return 0.0;
}

void LikelihoodFieldProbModel::doSetup(ros::NodeHandle &nh)
{

}
