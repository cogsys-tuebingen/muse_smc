#pragma once

#include <memory>
#include <vector>
#include <tf/tf.h>

#include <muse_amcl/pf/particle_set.hpp>

namespace muse_amcl {
struct Propagation {
    typedef std::shared_ptr<Propagation> Ptr;

    virtual void apply(ParticleSet::PoseIterator set) = 0;
};
}
