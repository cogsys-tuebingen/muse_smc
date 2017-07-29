#ifndef MAJORMAPAA2D_H
#define MAJORMAPAA2D_H

#include <muse_mcl/pose_samplers/uniform.hpp>
#include <muse_mcl/sampling/sampling_uniform.hpp>

namespace muse_mcl {
class UniformPrimaryMap2D : public SamplingUniform
{
public:
    UniformPrimaryMap2D() = default;

    virtual bool update(const std::string &frame) override;
    virtual void apply(ParticleSet &particle_set) override;
    virtual void apply(Particle &particle) override;

protected:
    using Metric   = muse_mcl::pose_generation::Metric;
    using Radian   = muse_mcl::pose_generation::Radian;
    using RandomPoseGenerator = muse_mcl::pose_generation::Uniform<Metric, Metric, Radian>;

    ProviderMap::Ptr primary_map_provider_;
    int              random_seed_;

    RandomPoseGenerator::Ptr   rng_;
    tf::Transform              w_T_primary_;
    tf::Transform              primary_T_o_;
    Map::ConstPtr primary_map;
    std::vector<Map::ConstPtr> secondary_maps_;
    std::vector<tf::Transform> secondary_maps_T_w_;


    virtual void doSetup(ros::NodeHandle &nh_private) override;
    virtual void doSetupMapProviders(ros::NodeHandle &nh_private,
                                     const MapProviders &map_providers) override;

};
}

#endif // MAJORMAPAA2D_H
