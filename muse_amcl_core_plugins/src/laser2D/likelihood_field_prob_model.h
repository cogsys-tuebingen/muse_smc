#ifndef LIKELIHOOD_FIELD_PROB_MODEL_H
#define LIKELIHOOD_FIELD_PROB_MODEL_H


#include <muse_amcl/particle_filter/update.hpp>

namespace muse_amcl {
class LikelihoodFieldProbModel : public Update
{
public:
    LikelihoodFieldProbModel();

    virtual double apply(const Data::ConstPtr &data,
                         const Map::ConstPtr &map,
                         ParticleSet::WeightIterator set) override;

protected:
    virtual void doSetup(ros::NodeHandle &nh) override;

};
}

#endif /* LIKELIHOOD_FIELD_PROB_MODEL_H */
