#ifndef PARTICLE_FILTER_HPP
#define PARTICLE_FILTER_HPP

#include "particle_set.hpp"
#include "update_queue.hpp"
#include "propagation_queue.hpp"

#include "../data_sources/tf_provider.hpp"

#include <memory>
#include <thread>

namespace muse_amcl {
class ParticleFilter {
public:
    ParticleFilter()
    {
    }

    void setup(const std::string &name,
               ros::NodeHandle &nh_private)
    {
        /// set the name
        name_ = name;

        int sample_size = 0;
        sample_size = nh_private.param("sample_size", sample_size);
        sample_size = nh_private.param("sample_size_max", sample_size);

        if(sample_size <= 0) {
            throw std::runtime_error("[ParticleFilter]: Either 'sample_size' or 'sample_size_max' must be defined and positive!");
        }
        int sample_size_min = 0;
        if(nh_private.getParam(parameter("sample_size_min"), sample_size_min)) {
            if(sample_size <= 0)
                throw std::runtime_error("[ParticleFilter]: 'sample_size_min' must be greater zero!");
            if(sample_size > sample_size)
                throw std::runtime_error("[ParticleFilter]: 'sample_size_min' must be less equal 'sample_size'!");
        }

        std::string frame_id;
        if(!nh_private.getParam(parameter("frame_id"), frame_id)) {
            throw std::runtime_error("[ParticleFilter]: 'frame_id' must be defined!");
        }

        particle_set_.reset(new ParticleSet(frame_id, sample_size, sample_size_min, sample_size));
    }


protected:
    std::string         name_;
    ParticleSet::Ptr    particle_set_;



    std::string parameter(const std::string &name)
    {
        return name_ + "/" + name;
    }


};
}

#endif // PARTICLE_FILTER_HPP
