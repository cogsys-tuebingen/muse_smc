#include "uniform_primary_map_2d.h"

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(muse_amcl::UniformPrimaryMap2D, muse_amcl::UniformSampling)

#include <muse_amcl/pose_generators/uniform.hpp>
#include <tf/tf.h>

using namespace muse_amcl;

using Metric   = muse_amcl::pose_generation::Metric;
using Radian   = muse_amcl::pose_generation::Radian;
using RandomPoseGenerator = muse_amcl::pose_generation::Uniform<Metric, Metric, Radian>;

void UniformPrimaryMap2D::apply(ParticleSet &particle_set)
{
    const ros::Time   now = ros::Time::now();
    const std::string frame = particle_set.getFrame();

    Map::ConstPtr              primary_map;
    std::vector<Map::ConstPtr> secondary_maps;
    tf::Transform              w_T_primary;
    tf::Transform              primary_T_o(primary_map->getOrigin().tf());
    std::vector<tf::Transform> secondary_map_transforms;

    if(!tf_provider_->lookupTransform(frame, primary_map->getFrame(), now, w_T_primary, tf_timeout_)) {
        throw std::runtime_error("[UniformPrimaryMap2D]: Could not get primary map transform!");
    }

    for(auto &m : map_providers_) {
        tf::StampedTransform map_transform;
        Map::ConstPtr map = m->getMap();
        if(tf_provider_->lookupTransform(map->getFrame(), frame, now, map_transform, tf_timeout_)) {
            secondary_maps.emplace_back(map);
            secondary_map_transforms.emplace_back(map_transform);
        }
    }


    //    // @TODO: remove random seed = 0
    /// particles are generated in the primary map frame, since formulation has
    /// to be axis-aligned, relative to the map origin

    math::Point min = primary_map->getMin();
    math::Point max = primary_map->getMax();
    {
        w_T_primary = w_T_primary * primary_T_o;
        min = primary_map->getOrigin().tf().inverse() * min;
        max = primary_map->getOrigin().tf().inverse() * max;
    }
    RandomPoseGenerator  rng({min.x(), min.y(), 0.0}, {max.x(), max.y(), 2 * M_PI}, 0);
    particle_set.resize(sample_size_);

    ParticleSet::Particles &particles = particle_set.getParticles();

    const ros::Time sampling_start = ros::Time::now();
    double sum_weight = 0.0;
    for(auto &particle : particles) {
        bool valid = false;
        while(!valid) {
            ros::Time now = ros::Time::now();
            if(sampling_start + sampling_timeout_ < now) {
                std::cerr << "[UniformPrimaryMap2D]: Sampling timed out!" << std::endl;
                break;
            }

            math::Pose pose = primary_T_o * math::Pose(rng());
            particle.pose_  = w_T_primary * pose;

            sum_weight += particle.weight_;
            valid = primary_map->valid(pose);
            if(valid) {
                for(const auto &m : secondary_maps) {
                    valid &= m->valid(particle.pose_);
                }
            }
        }
    }
    particle_set.normalize(sum_weight);
}

void UniformPrimaryMap2D::doSetup(ros::NodeHandle &nh_private)
{
}

void UniformPrimaryMap2D::doSetupMapProviders(ros::NodeHandle &nh_private,
                                              const MapProviders &map_providers)
{
    std::string primary_map_provider = nh_private.param(parameter("primary_map"), std::string(""));
    std::vector<std::string> secondary_map_providers;
    nh_private.getParam(parameter("secondary_maps"), secondary_map_providers);

    if(primary_map_provider == "")
        throw std::runtime_error("[UniformPrimaryMap2D]: Primary map provider must be set!");

    primary_map_provider_ = map_providers.at(primary_map_provider);
    for(auto m : secondary_map_providers) {
        map_providers_.emplace_back(map_providers.at(m));
    }

}
