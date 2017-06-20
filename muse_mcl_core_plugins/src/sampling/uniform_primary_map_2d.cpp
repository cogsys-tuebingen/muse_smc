#include "uniform_primary_map_2d.h"

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(muse_mcl::UniformPrimaryMap2D, muse_mcl::UniformSampling)

#include <tf/tf.h>

using namespace muse_mcl;


bool UniformPrimaryMap2D::update(const std::string &frame)
{
    const ros::Time   now = ros::Time::now();

    primary_map  = primary_map_provider_->getMap();
    if(!primary_map) {
        Logger::getLogger().error("Primary provider '" + primary_map_provider_->getName() + "' has not a map yet." , "UniformSampling:" + name_);
        return false;
    }

    primary_T_o_ = primary_map->getOrigin().getPose();
    if(!tf_provider_->lookupTransform(frame, primary_map->getFrame(), now, w_T_primary_, tf_timeout_)) {
        return false;
    }

    const std::size_t map_provider_count = map_providers_.size();
    for(std::size_t i = 0 ; i < map_provider_count ; ++i) {
        Map::ConstPtr map = map_providers_[i]->getMap();
        if(!map) {
            Logger::getLogger().error("Secondary provider '" + map_providers_[i]->getName() + "' has not a map yet." , "UniformSampling:" + name_);
            return false;
        }

        if(tf_provider_->lookupTransform(map->getFrame(), frame, now, secondary_maps_T_w_[i], tf_timeout_)) {
            secondary_maps_[i] = map;
        } else {
             return false;
        }
    }
    /// particles are generated in the primary map frame, since formulation has
    /// to be axis-aligned, relative to the map origin
    /// but internal frames are already within calculation

    math::Point min = primary_map->getMin();
    math::Point max = primary_map->getMax();
    {
        auto o_T_primary = primary_T_o_.inverse();
        min = o_T_primary * min;
        max = o_T_primary * max;
    }
    rng_.reset(new RandomPoseGenerator({min.x(), min.y(), -M_PI}, {max.x(), max.y(), M_PI}));
    if(random_seed_ >= 0) {
        rng_.reset(new RandomPoseGenerator({min.x(), min.y(), -M_PI}, {max.x(), max.y(), M_PI}, random_seed_));
    }

    return true;
}

void UniformPrimaryMap2D::apply(ParticleSet &particle_set)
{
    if(sample_size_ < particle_set.getSampleSizeMinimum() ||
            sample_size_ > particle_set.getSampleSizeMaximum()) {
        Logger::getLogger().error("Initialization sample size invalid.", "UniformSampling:" + name_);
        throw std::runtime_error("Initialization sample size invalid!");
    }

    if(!update(particle_set.getFrame()))
        return;


    Insertion insertion = particle_set.getInsertion();
    const std::size_t          secondary_maps_count = secondary_maps_.size();
    const ros::Time sampling_start = ros::Time::now();
    Particle particle;
    for(std::size_t i = 0 ; i < sample_size_; ++i) {
        bool valid = false;
        while(!valid) {
            ros::Time now = ros::Time::now();
            if(sampling_start + sampling_timeout_ < now) {
                Logger::getLogger().error("Sampling timed out.", "UniformSampling:" + name_);
                break;
            }

            math::Pose pose = primary_T_o_ * math::Pose(rng_->get());
            valid = primary_map->validate(pose);
            if(valid) {
                particle.pose_  = w_T_primary_ * pose;
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
            Logger::getLogger().error("Sampling timed out.", "UniformSampling:" + name_);
            break;
        }

        math::Pose pose = primary_T_o_ * math::Pose(rng_->get());
        valid = primary_map->validate(pose);
        if(valid) {
            particle.pose_  = w_T_primary_ * pose;
            for(std::size_t i = 0 ; i < secondary_maps_count ; ++i) {
                valid &= secondary_maps_[i]->validate(secondary_maps_T_w_[i] * particle.pose_);
            }
        }
    }
}

void UniformPrimaryMap2D::doSetup(ros::NodeHandle &nh_private)
{
    random_seed_ = nh_private.param(parameter("seed"), -1);

    Logger &l = Logger::getLogger();
    l.info("random_seed_='" + std::to_string(random_seed_) + "'", "UniformSampling:" + name_);
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
    std::string ms ="[";
    for(auto m : secondary_map_providers) {
        map_providers_.emplace_back(map_providers.at(m));
        ms += m + ",";
    }
    ms.back() = ']';

    secondary_maps_T_w_.resize(secondary_map_providers.size());
    secondary_maps_.resize(secondary_map_providers.size());

    Logger &l = Logger::getLogger();
    l.info("primary_map_provider='" + primary_map_provider + "'", "UniformSampling:" + name_);
    l.info("secondary_maps='" + ms + "'", "UniformSampling:" + name_);

}
