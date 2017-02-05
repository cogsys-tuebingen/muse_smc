#include "normal_2d.h"

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(muse_amcl::Normal2D, muse_amcl::NormalSampling)

#include <ros/time.h>
#include <muse_amcl/pose_samplers/normal.hpp>

using namespace muse_amcl;

using Metric              = muse_amcl::pose_generation::Metric;
using Radian              = muse_amcl::pose_generation::Radian;
using RandomPoseGenerator = muse_amcl::pose_generation::Normal<Metric, Metric, Radian>;


void Normal2D::update(const std::string &frame)
{
    const ros::Time   now = ros::Time::now();
    for(auto &m : map_providers_) {
        tf::StampedTransform map_T_w;
        Map::ConstPtr map = m->getMap();
        if(tf_provider_->lookupTransform(map->getFrame(), frame, now, map_T_w, tf_timeout_)) {
            maps_.emplace_back(map);
            maps_T_w_.emplace_back(map_T_w);
        } else {
            throw std::runtime_error("[Normal2D]: Could not lookup transform '" + frame + " -> " + map->getFrame() + "'!");
        }
    }
}

void Normal2D::apply(const math::Pose       &pose,
                     const math::Covariance &covariance,
                     ParticleSet            &particle_set)
{
    RandomPoseGenerator::Ptr rng(new RandomPoseGenerator(pose.getEigen3D(), covariance.getEigen3D()));
    if(random_seed_ >= 0) {
        rng.reset(new RandomPoseGenerator(pose.getEigen3D(), covariance.getEigen3D(), random_seed_));
    }

    if(sample_size_ < particle_set.getSampleSizeMinimum() &&
            sample_size_ > particle_set.getSampleSizeMaximum()) {
        throw std::runtime_error("Initialization sample size invalid!");
    }

    Insertion insertion = particle_set.getInsertion();

    const ros::Time sampling_start = ros::Time::now();
    Particle particle;
    for(std::size_t i = 0 ; i < sample_size_ ; ++i) {
        bool valid = false;
        while(!valid) {
            ros::Time now = ros::Time::now();
            if(sampling_start + sampling_timeout_ < now) {
                std::cerr << "[Normal2D]: Sampling timed out!" << std::endl;
                break;
            }

            particle.pose_ = rng->get();
            valid = true;
            for(const auto &m : maps_) {
                valid &= m->validate(particle.pose_);
            }
        }
        insertion.insert(particle);
    }
}

void Normal2D::doSetup(ros::NodeHandle &nh_private)
{
    random_seed_ = nh_private.param(parameter("seed"), -1);
}
