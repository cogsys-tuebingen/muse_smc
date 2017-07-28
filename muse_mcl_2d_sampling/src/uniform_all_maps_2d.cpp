#include "uniform_all_maps_2d.h"

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(muse_mcl::UniformAllMaps2D, muse_mcl::SamplingUniform)


#include <tf/tf.h>
#include <eigen3/Eigen/Core>

using namespace muse_mcl;

UniformAllMaps2D::UniformAllMaps2D()
{
}

bool UniformAllMaps2D::update(const std::string &frame)
{
    const ros::Time   now   = ros::Time::now();

    muse_mcl::math::Point min(std::numeric_limits<double>::max(),
                               std::numeric_limits<double>::max());
    muse_mcl::math::Point max(std::numeric_limits<double>::lowest(),
                               std::numeric_limits<double>::lowest());

    const std::size_t map_provider_count = map_providers_.size();
    maps_T_w_.resize(map_provider_count);
    maps_.resize(map_provider_count);
    for(std::size_t i = 0 ; i < map_provider_count ; ++i) {
        Map::ConstPtr map = map_providers_[i]->getMap();
        if(!map) {
            throw std::runtime_error("[Normal2D] : map was null!");
            continue;
        }
        if(tf_provider_->lookupTransform(map->getFrame(), frame, now, maps_T_w_[i], tf_timeout_)) {
            maps_[i] = map;

            tf::Transform w_T_map = maps_T_w_[i].inverse();
            min = min.cwiseMin(w_T_map * map->getMin());
            max = max.cwiseMax(w_T_map * map->getMax());
        } else {
            return false;
        }
    }

    rng_.reset(new RandomPoseGenerator({min.x(), min.y(), -M_PI}, {max.x(), max.y(), M_PI}));
    if(random_seed_ >= 0) {
        rng_.reset(new RandomPoseGenerator({min.x(), min.y(), -M_PI}, {max.x(), max.y(), M_PI}, 0));
    }

    return true;
}

void UniformAllMaps2D::apply(ParticleSet &particle_set)
{
    if(sample_size_ < particle_set.getSampleSizeMinimum() ||
            sample_size_ > particle_set.getSampleSizeMaximum()) {
        throw std::runtime_error("Initialization sample size invalid!");
    }

    if(!update(particle_set.getFrame()))
        return;

    Insertion insertion = particle_set.getInsertion();
    const ros::Time sampling_start = ros::Time::now();
    const std::size_t map_count = maps_.size();

    Particle particle;
    particle.weight_ = 1.0 / static_cast<double>(sample_size_);
    for(std::size_t i = 0 ; i < sample_size_ ; ++i) {
        bool valid = false;
        while(!valid) {
            ros::Time now = ros::Time::now();
            if(sampling_start + sampling_timeout_ < now) {
                return;
            }


            particle.pose_ = rng_->get();
            valid = true;
            for(std::size_t i = 0 ; i < map_count ; ++i) {
                valid &= maps_[i]->validate(maps_T_w_[i] * particle.pose_);
            }
        }
        insertion.insert(particle);
    }
}

void UniformAllMaps2D::apply(Particle &particle)
{
    const ros::Time sampling_start = ros::Time::now();
    const std::size_t map_count = maps_.size();
    bool valid = false;
    while(!valid) {
        ros::Time now = ros::Time::now();
        if(sampling_start + sampling_timeout_ < now) {
            break;
        }

        particle.pose_ = rng_->get();
        valid = true;
        for(std::size_t i = 0 ; i < map_count ; ++i) {
            valid &= maps_[i]->validate(maps_T_w_[i] * particle.pose_);
        }
    }
}

void UniformAllMaps2D::doSetup(ros::NodeHandle &nh_private)
{
    random_seed_ = nh_private.param(parameter("seed"), -1);
}
