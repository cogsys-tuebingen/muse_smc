#include "uniform_primary_map_2d.h"

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(muse_amcl::UniformPrimaryMap2D, muse_amcl::UniformSampling)

#include <tf/tf.h>

using namespace muse_amcl;


void UniformPrimaryMap2D::update(const std::string &frame)
{
    const ros::Time   now = ros::Time::now();

    primary_map = primary_map_provider_->getMap();
    primary_T_o = primary_map->getOrigin().tf();
    if(!tf_provider_->lookupTransform(frame, primary_map->getFrame(), now, w_T_primary, tf_timeout_)) {
        throw std::runtime_error("[UniformPrimaryMap2D]: Could not get primary map transform!");
    }

    for(auto &m : map_providers_) {
        tf::StampedTransform map_T_w;
        Map::ConstPtr map = m->getMap();
        if(tf_provider_->lookupTransform(map->getFrame(), frame, now, map_T_w, tf_timeout_)) {
            secondary_maps_.emplace_back(map);
            secondary_maps_T_w_.emplace_back(map_T_w);
        } else {
            std::cerr << "[UniformPrimaryMap2D]: Could not lookup transform '"
                      << frame << " -> " << map->getFrame()
                      << std::endl;
        }
    }
    /// particles are generated in the primary map frame, since formulation has
    /// to be axis-aligned, relative to the map origin
    /// but internal frames are already within calculation

    math::Point min = primary_map->getMin();
    math::Point max = primary_map->getMax();
    {
        auto o_T_primary = primary_T_o.inverse();
        min = o_T_primary * min;
        max = o_T_primary * max;
    }
    rng_.reset(new RandomPoseGenerator({min.x(), min.y(), -M_PI}, {max.x(), max.y(), M_PI}));
    if(random_seed_ >= 0) {
        rng_.reset(new RandomPoseGenerator({min.x(), min.y(), -M_PI}, {max.x(), max.y(), M_PI}, random_seed_));
    }
}

void UniformPrimaryMap2D::apply(ParticleSet &particle_set)
{
    if(sample_size_ < particle_set.getSampleSizeMinimum() ||
            sample_size_ > particle_set.getSampleSizeMaximum()) {
        throw std::runtime_error("Initialization sample size invalid!");
    }

    ParticleSet::Insertion insertion = particle_set.getInsertion();
    const std::size_t          secondary_maps_count = secondary_maps_.size();
    const ros::Time sampling_start = ros::Time::now();
    Particle particle;
    for(std::size_t i = 0 ; i < sample_size_; ++i) {
        bool valid = false;
        while(!valid) {
            ros::Time now = ros::Time::now();
            if(sampling_start + sampling_timeout_ < now) {
                std::cerr << "[UniformPrimaryMap2D]: Sampling timed out!" << std::endl;
                break;
            }

            math::Pose pose = primary_T_o * math::Pose(rng_->get());
            valid = primary_map->validate(pose);
            if(valid) {
                particle.pose_  = w_T_primary * pose;
                for(std::size_t i = 0 ; i < secondary_maps_count ; ++i) {
                    valid &= secondary_maps_[i]->validate(secondary_maps_T_w_[i] * particle.pose_);
                }
            }
        }
        insertion.insert(particle);
    }
}

void UniformPrimaryMap2D::apply(Particle &particle)
{
    const std::size_t secondary_maps_count = secondary_maps_.size();
    const ros::Time   sampling_start = ros::Time::now();
    bool valid = false;
    while(!valid) {
        ros::Time now = ros::Time::now();
        if(sampling_start + sampling_timeout_ < now) {
            std::cerr << "[UniformPrimaryMap2D]: Sampling timed out!" << std::endl;
            break;
        }

        math::Pose pose = primary_T_o * math::Pose(rng_->get());
        valid = primary_map->validate(pose);
        if(valid) {
            particle.pose_  = w_T_primary * pose;
            for(std::size_t i = 0 ; i < secondary_maps_count ; ++i) {
                valid &= secondary_maps_[i]->validate(secondary_maps_T_w_[i] * particle.pose_);
            }
        }
    }
}

void UniformPrimaryMap2D::doSetup(ros::NodeHandle &nh_private)
{
    random_seed_ = nh_private.param(parameter("seed"), -1);
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
