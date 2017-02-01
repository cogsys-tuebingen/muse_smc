#include "uniform_all_maps_2d.h"

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(muse_amcl::UniformAllMaps2D, muse_amcl::UniformSampling)


#include <tf/tf.h>
#include <eigen3/Eigen/Core>

using namespace muse_amcl;

UniformAllMaps2D::UniformAllMaps2D()
{
}

void UniformAllMaps2D::update(const std::string &frame)
{
    const ros::Time   now   = ros::Time::now();

    muse_amcl::math::Point min(std::numeric_limits<double>::max(),
                               std::numeric_limits<double>::max());
    muse_amcl::math::Point max(std::numeric_limits<double>::lowest(),
                               std::numeric_limits<double>::lowest());

    for(auto &m : map_providers_) {
        tf::Transform map_T_w;
        Map::ConstPtr map = m->getMap();
        if(tf_provider_->lookupTransform(map->getFrame(), frame, now, map_T_w, tf_timeout_)) {
            maps_.emplace_back(map);
            maps_T_w_.emplace_back(map_T_w);

            tf::Transform w_T_map = map_T_w.inverse();
            min = min.cwiseMin(w_T_map * map->getMin());
            max = max.cwiseMax(w_T_map * map->getMax());

        } else {
            std::cerr << "[UniformAllMaps2D]: Could not lookup transform '"
                      << frame << " -> " << map->getFrame()
                      << std::endl;
        }
    }

    rng_.reset(new RandomPoseGenerator({min.x(), min.y(), -M_PI}, {max.x(), max.y(), M_PI}));
    if(random_seed_ >= 0) {
        rng_.reset(new RandomPoseGenerator({min.x(), min.y(), -M_PI}, {max.x(), max.y(), M_PI}, 0));
    }

}

void UniformAllMaps2D::apply(ParticleSet &particle_set)
{
    if(sample_size_ < particle_set.getSampleSizeMinimum() ||
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
                std::cerr << "[UniformAllMaps2D]: Sampling timed out!" << std::endl;
                break;
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
            std::cerr << "[UniformAllMaps2D]: Sampling timed out!" << std::endl;
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
