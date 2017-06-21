#include "normal_2d.h"

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(muse_mcl::Normal2D, muse_mcl::NormalSampling)

#include <ros/time.h>
#include <muse_mcl/pose_samplers/normal.hpp>

using namespace muse_mcl;

using Metric              = muse_mcl::pose_generation::Metric;
using Radian              = muse_mcl::pose_generation::Radian;
using RandomPoseGenerator = muse_mcl::pose_generation::Normal<Metric, Metric, Radian>;


void Normal2D::update(const std::string &frame)
{
    const ros::Time   now   = ros::Time::now();

    muse_mcl::math::Point min(std::numeric_limits<double>::max(),
                               std::numeric_limits<double>::max());
    muse_mcl::math::Point max(std::numeric_limits<double>::lowest(),
                               std::numeric_limits<double>::lowest());

    for(auto &m : map_providers_) {
        tf::Transform map_T_w;
        Map::ConstPtr map = m->getMap();
        if(!map) {
            throw std::runtime_error("[Normal2D] : map was null!");
        }

        if(tf_provider_->lookupTransform(map->getFrame(), frame, now, map_T_w, tf_timeout_)) {
            maps_.emplace_back(map);
            maps_T_w_.emplace_back(map_T_w);

            tf::Transform w_T_map = map_T_w.inverse();
            min = min.cwiseMin(w_T_map * map->getMin());
            max = max.cwiseMax(w_T_map * map->getMax());

        }
    }
}

void Normal2D::apply(const math::Pose       &pose,
                     const math::Covariance &covariance,
                     ParticleSet            &particle_set)
{
    update(particle_set.getFrame());

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
    const std::size_t map_count = maps_.size();

    Particle particle;
    for(std::size_t i = 0 ; i < sample_size_ ; ++i) {
        bool valid = false;
        while(!valid) {
            ros::Time now = ros::Time::now();
            if(sampling_start + sampling_timeout_ < now) {
                return;
            }

            particle.pose_ = rng->get();
            valid = true;
            for(std::size_t i = 0 ; i < map_count ; ++i) {
                valid &= maps_[i]->validate(maps_T_w_[i] * particle.pose_);
            }
        }
        insertion.insert(particle);
    }
}

void Normal2D::doSetup(ros::NodeHandle &nh_private)
{
    random_seed_ = nh_private.param(parameter("seed"), -1);
}
