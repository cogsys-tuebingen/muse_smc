#include "uniform_main_map_2d.h"

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(muse_amcl::UniformMainMap2D, muse_amcl::UniformPoseGeneration)

#include <muse_amcl/pose_generators/uniform.hpp>

using namespace muse_amcl;

using Metric   = muse_amcl::pose_generation::Metric;
using Radian   = muse_amcl::pose_generation::Radian;
using RandomPoseGenerator = muse_amcl::pose_generation::Uniform<Metric, Metric, Radian>;

void UniformMainMap2D::apply(ParticleSet &particle_set)
{
//    std::vector<Map::ConstPtr>        maps;
//    std::vector<tf::StampedTransform> map_transforms;
//    const ros::Time   now = ros::Time::now();
//    const std::string frame = particle_set.getFrame();
//    for(auto &m : maps_providers_) {
//        tf::StampedTransform map_transform;
//        Map::ConstPtr map = m->getMap();
//        if(tf_->lookupTransform(map->getFrame(), frame, now, map_transform, tf_timeout_)) {
//            maps.emplace_back(map);
//            map_transforms.emplace_back(map_transform);
//        }
//    }


//    // @TODO: remove random seed = 0
//    RandomPoseGenerator  rng(pose.eigen3D(), covariance.eigen3D(), 0);
//    particle_set.resize(sample_size_);

//    ParticleSet::Particles &particles = particle_set.getParticles();

//    const ros::Time sampling_start = ros::Time::now();
//    double sum_weight = 0.0;
//    for(auto &particle : particles) {
//        bool valid = false;
//        while(!valid) {
//            ros::Time now = ros::Time::now();
//            if(sampling_start + sampling_timeout_ < now)
//                break;

//            particle.pose_ = rng();
//            sum_weight += particle.weight_;
//            valid = true;
//            for(const auto &m : maps) {
//                valid &= m->valid(particle.pose_);
//            }
//        }
//    }
//    particle_set.normalize(sum_weight);
}

void UniformMainMap2D::doSetup(ros::NodeHandle &nh_private)
{

}
