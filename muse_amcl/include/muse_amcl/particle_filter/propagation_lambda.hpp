#pragma once

#include <chrono>
#include <memory>
#include <functional>
#include "particle_set.hpp"

namespace muse_amcl {
class PropagationLambda {
public:
    typedef std::shared_ptr<PropagationLambda> Ptr;

    struct Less {
        bool operator()( const PropagationLambda& lhs,
                         const PropagationLambda& rhs ) const
        {
            return lhs.stamp() > rhs.stamp();
        }
    };

    PropagationLambda(std::function<void (ParticleSet::PoseIterator set)> lambda,
                      std::chrono::time_point<std::chrono::system_clock>   stamp) :
        lambda_(lambda),
        stamp_(stamp)
    {
    }

    inline void operator ()(ParticleSet::PoseIterator set)
    {
        lambda_(set);
    }

    inline std::chrono::time_point<std::chrono::system_clock> stamp() const
    {
        return stamp_;
    }

private:
    std::function<void (ParticleSet::PoseIterator set)>  lambda_;
    std::chrono::time_point<std::chrono::system_clock>    stamp_;

};
}
