#pragma once

#include <chrono>
#include <functional>
#include "particle_set.hpp"

namespace muse_amcl {
class UpdateLambda {
public:
    UpdateLambda(std::function<void(ParticleSet::WeightIterator set)> lambda,
                 std::chrono::time_point<std::chrono::system_clock>   stamp) :
        lambda_(lambda),
        stamp_(stamp)
    {
    }

    inline void operator ()()
    {
        lambda_();
    }

    inline std::chrono::time_point<std::chrono::system_clock> stamp() const
    {
        return stamp_;
    }

private:
    std::function<void(ParticleSet::WeightIterator set)>  lambda_;
    std::chrono::time_point<std::chrono::system_clock>    stamp_;

};
}
