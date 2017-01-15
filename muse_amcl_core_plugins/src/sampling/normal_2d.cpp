#include "normal_2d.h"

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(muse_amcl::Normal2D, muse_amcl::NormalSampling)

#include <muse_amcl/pose_samplers/normal.hpp>

#include <ros/time.h>

using namespace muse_amcl;

using Metric              = muse_amcl::pose_generation::Metric;
using Radian              = muse_amcl::pose_generation::Radian;
using RandomPoseGenerator = muse_amcl::pose_generation::Normal<Metric, Metric, Radian>;

void Normal2D::apply(const math::Pose       &pose,
                     const math::Covariance &covariance,
                     ParticleSet            &particle_set)
{
    std::vector<Map::ConstPtr>        maps;
    std::vector<tf::StampedTransform> maps_T_w;
    const ros::Time   now = ros::Time::now();
    const std::string world_frame = particle_set.getFrame();
    for(auto &m : map_providers_) {
        tf::StampedTransform map_T_w;
        Map::ConstPtr map = m->getMap();
        if(tf_provider_->lookupTransform(map->getFrame(), world_frame, now, map_T_w, tf_timeout_)) {
            maps.emplace_back(map);
            maps_T_w.emplace_back(map_T_w);
        } else {
            throw std::runtime_error("[Normal2D]: Could not lookup transform '" + world_frame + " -> " + map->getFrame() + "'!");
        }
    }

    RandomPoseGenerator::Ptr  rng(new RandomPoseGenerator(pose.eigen3D(), covariance.eigen3D()));
    if(random_seed_ >= 0) {
        rng.reset(new RandomPoseGenerator(pose.eigen3D(), covariance.eigen3D(), random_seed_));
    }

    ParticleSet::Particles &particles = particle_set.getParticles();
    particles.resize(sample_size_);

    const ros::Time sampling_start = ros::Time::now();
    double sum_weight = 0.0;
    for(auto &particle : particles) {
        bool valid = false;
        while(!valid) {
            ros::Time now = ros::Time::now();
            if(sampling_start + sampling_timeout_ < now) {
                std::cerr << "[Normal2D]: Sampling timed out!" << std::endl;
                break;
            }

            particle.pose_ = rng->get();
            sum_weight += particle.weight_;
            valid = true;
            for(const auto &m : maps) {
                valid &= m->validate(particle.pose_);
            }
        }
    }
    particle_set.normalize(sum_weight);
}

void Normal2D::doSetup(ros::NodeHandle &nh_private)
{
    random_seed_ = nh_private.param(parameter("seed"), -1);
}
