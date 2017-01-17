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

        int sample_size;
        if(!nh_private.getParam(parameter("sample_size"), sample_size)) {
            throw std::runtime_error("[ParticleFilter]: 'sample_size' must be defined!");
        }
        std::size_t sample_size_min = nh_private.param(parameter("sample_size_min"), sample_size);
        std::size_t sample_size_max = nh_private.param(parameter("sample_size_max"), sample_size);
        if(sample_size_min > sample_size_max) {
            throw std::runtime_error("[ParticleFilter]: 'sample_size_mim' cannot be greater than 'sample_size_max'!");
        }
        std::string frame_id;
        if(!nh_private.getParam(parameter("frame_id"), frame_id)) {
            throw std::runtime_error("[ParticleFilter]: 'frame_id' must be defined!");
        }

        assert(sample_size     > 0);
        assert(sample_size_min > 0);
        assert(sample_size_max > 0);
        particle_set_.reset(new ParticleSet(frame_id, sample_size, sample_size_min, sample_size_max));
    }


protected:
    std::string      name_;
    ParticleSet::Ptr particle_set_;

    std::string parameter(const std::string &name)
    {
        return name_ + "/" + name;
    }


};
}

#endif // PARTICLE_FILTER_HPP
