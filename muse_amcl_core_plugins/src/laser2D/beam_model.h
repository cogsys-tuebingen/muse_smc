#ifndef BEAM_MODEL_H
#define BEAM_MODEL_H

#include <muse_amcl/particle_filter/update.hpp>

namespace muse_amcl {
class BeamModel : public UpdateModel
{
public:
    BeamModel();

    virtual void update(const Data::ConstPtr &data,
                        const Map::ConstPtr &map,
                        ParticleSet::Weights set) override;

protected:
    virtual void doSetup(ros::NodeHandle &nh) override;


};
}

#endif /* BEAM_MODEL_H */
