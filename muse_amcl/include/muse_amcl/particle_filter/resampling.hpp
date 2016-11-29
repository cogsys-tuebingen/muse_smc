#ifndef RESAMPLING_HPP
#define RESAMPLING_HPP

#include <memory>
#include <vector>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <ros/node_handle.h>

#include <muse_amcl/particle_filter/particle_set.hpp>
#include "../particle_filter/particle_set.hpp"

namespace muse_amcl {
class Resampling {
public:
    typedef std::shared_ptr<Resampling> Ptr;

    Resampling(ParticleSet &particle_set,
               const std::size_t min_size,
               const std::size_t max_size) :
        particle_set_(particle_set),
        min_size_(min_size),
        max_size_(max_size)
    {
    }

    void resample() = 0;

private:
    ParticleSet &particle_set_;
    const std::size_t min_size_;    /// minimal size of the set
    const std::size_t max_size_;    /// maximal size of the set / default size
};
}

#endif /* RESAMPLING_HPP */
