#pragma once

#include <queue>
#include <functional>
#include <memory>

#include "../pf/particle_set.hpp"

namespace muse_amcl {
class UpdateQueue {
public:
    typedef std::function<void(ParticleSet::WeightIterator)> UpdateLambda;
    typedef std::shared_ptr<UpdateQueue> Ptr;

private:
    std::priority_queue<UpdateLambda> q_;


    /// condition variable

};
}
