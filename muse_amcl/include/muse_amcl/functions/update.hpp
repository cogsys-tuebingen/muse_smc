#pragma once

#include <memory>
#include <vector>
#include <tf/tf.h>

#include <muse_amcl/pf/particle_set.hpp>

namespace muse_amcl {
struct Update {
    typedef std::shared_ptr<Update> Ptr;

    virtual double apply(ParticleSet::WeightIterator set) = 0;
};
}
