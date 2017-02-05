#ifndef LIKELIHOOD_FIELD_PROB_MODEL_H
#define LIKELIHOOD_FIELD_PROB_MODEL_H


#include <muse_amcl/particle_filter/update.hpp>

namespace muse_amcl {
class LikelihoodFieldProbModel : public UpdateModel
{
public:
    LikelihoodFieldProbModel();

    virtual void update(const Data::ConstPtr &data,
                        const Map::ConstPtr &map,
                        ParticleSet::Weights set) override;

protected:
    std::size_t max_beams_;
    double      z_hit_;
    double      z_rand_;
    double      sigma_hit_;
    double      denominator_hit_;
    bool        beam_skip_;
    double      beam_skip_distance_;
    double      beam_skip_threshold_;
    double      beam_skip_error_threshold_;

    std::vector<int>    observation_histogram_;
    std::vector<char>   observation_mask_;
    std::vector<double> observation_probability_buffer_;

    virtual void doSetup(ros::NodeHandle &nh_private) override;

};
}

#endif /* LIKELIHOOD_FIELD_PROB_MODEL_H */
