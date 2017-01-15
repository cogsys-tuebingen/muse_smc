#include "uniform_all_maps_2d.h"

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(muse_amcl::UniformAllMaps2D, muse_amcl::UniformSampling)

#include <muse_amcl/pose_samplers/uniform.hpp>
#include <tf/tf.h>
#include <eigen3/Eigen/Core>

using namespace muse_amcl;

using Metric              = muse_amcl::pose_generation::Metric;
using Radian              = muse_amcl::pose_generation::Radian;
using RandomPoseGenerator = muse_amcl::pose_generation::Uniform<Metric, Metric, Radian>;

UniformAllMaps2D::UniformAllMaps2D()
{
}

void UniformAllMaps2D::apply(ParticleSet &particle_set)
{
    std::vector<Map::ConstPtr> maps;
    std::vector<tf::Transform> maps_T_w;
    const ros::Time   now   = ros::Time::now();
    const std::string world_frame = particle_set.getFrame();

    muse_amcl::math::Point min(std::numeric_limits<double>::max(),
                               std::numeric_limits<double>::max());
    muse_amcl::math::Point max(std::numeric_limits<double>::lowest(),
                               std::numeric_limits<double>::lowest());

    for(auto &m : map_providers_) {
        tf::Transform map_T_w;
        Map::ConstPtr map = m->getMap();
        if(tf_provider_->lookupTransform(map->getFrame(), world_frame, now, map_T_w, tf_timeout_)) {
            maps.emplace_back(map);
            maps_T_w.emplace_back(map_T_w);

            tf::Transform w_T_map = map_T_w.inverse();
            min = min.cwiseMin(w_T_map * map->getMin());
            max = max.cwiseMax(w_T_map * map->getMax());

        } else {
            std::cerr << "[UniformAllMaps2D]: Could not lookup transform '"
                      << world_frame << " -> " << map->getFrame()
                      << std::endl;
        }
    }

    RandomPoseGenerator::Ptr rng(new RandomPoseGenerator({min.x(), min.y(), -M_PI}, {max.x(), max.y(), M_PI}));
    if(random_seed_ >= 0) {
        rng.reset(new RandomPoseGenerator({min.x(), min.y(), -M_PI}, {max.x(), max.y(), M_PI}, 0));
    }

    particle_set.resize(sample_size_);

    ParticleSet::Particles &particles = particle_set.getParticles();
    const ros::Time sampling_start = ros::Time::now();
    const std::size_t map_count = maps.size();
    double sum_weight = 0.0;
    for(auto &particle : particles) {
        bool valid = false;
        while(!valid) {
            ros::Time now = ros::Time::now();
            if(sampling_start + sampling_timeout_ < now) {
                std::cerr << "[UniformAllMaps2D]: Sampling timed out!" << std::endl;
                break;
            }

            particle.pose_ = rng->get();
            sum_weight += particle.weight_;
            valid = true;
            for(std::size_t i = 0 ; i < map_count ; ++i) {
                valid &= maps[i]->validate(maps_T_w[i] * particle.pose_);
            }
        }
    }
    particle_set.normalize(sum_weight);
}

void UniformAllMaps2D::doSetup(ros::NodeHandle &nh_private)
{
    random_seed_ = nh_private.param(parameter("seed"), -1);
}
