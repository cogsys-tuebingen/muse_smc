#ifndef LIKELIHOOD_FIELD_MODEL_H
#define LIKELIHOOD_FIELD_MODEL_H

#include <muse_amcl/particle_filter/update.hpp>

namespace muse_amcl {
class LikelihoodFieldModel : public UpdateModel
{
public:
    LikelihoodFieldModel();

    virtual void update(const Data::ConstPtr &data,
                        const Map::ConstPtr &map,
                        ParticleSet::Weights set) override;
protected:
    virtual void doSetup(ros::NodeHandle &nh) override;

};
}

#endif /* LIKELIHOOD_FIELD_MODEL_H */
