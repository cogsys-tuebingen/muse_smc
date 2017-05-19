#ifndef LIKELIHOOD_FIELD_AMCL_H
#define LIKELIHOOD_FIELD_AMCL_H

#include <muse_mcl/particle_filter/update.hpp>

namespace muse_mcl {
class LikelihoodFieldModelAMCL : public UpdateModel
{
public:
    LikelihoodFieldModelAMCL();

    virtual void update(const Data::ConstPtr &data,
                        const Map::ConstPtr &map,
                        ParticleSet::Weights set) override;
protected:
    std::size_t max_beams_;
    double      z_hit_;
    double      z_rand_;
    double      sigma_hit_;
    double      denominator_hit_;

    virtual void doSetup(ros::NodeHandle &nh_private) override;

};
}

#endif // LIKELIHOOD_FIELD_AMCL_H
