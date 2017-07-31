#ifndef BEAM_MODEL_AMCL_H
#define BEAM_MODEL_AMCL_H


#include <muse_mcl/update/update.hpp>

namespace muse_mcl {
class BeamModelAMCL : public ModelUpdate
{
public:
    BeamModelAMCL();

    virtual void update(const Data::ConstPtr &data,
                        const Map::ConstPtr &map,
                        ParticleSet::Weights set) override;

protected:
    std::size_t max_beams_;
    double      z_hit_;
    double      z_short_;
    double      z_max_;
    double      z_rand_;
    double      sigma_hit_;
    double      denominator_hit_;
    double      lambda_short_;
    double      chi_outlier_;

    virtual void doSetup(ros::NodeHandle &nh_private) override;

};
}
#endif // BEAM_MODEL_AMCL_H
