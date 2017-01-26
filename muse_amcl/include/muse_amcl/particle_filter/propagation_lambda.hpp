#ifndef PROPAGATION_LAMBDA_HPP
#define PROPAGATION_LAMBDA_HPP

#include <ros/time.h>
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

    PropagationLambda(std::function<void (ParticleSet::Poses set)> lambda,
                      const ros::Time &stamp) :
        lambda_(lambda),
        stamp_(stamp)
    {
    }

    inline void operator ()(ParticleSet::Poses set)
    {
        lambda_(set);
    }

    inline const ros::Time & stamp() const
    {
        return stamp_;
    }

private:
    std::function<void (ParticleSet::Poses set)>  lambda_;
    ros::Time                                     stamp_;

};
}

#endif /* PROPAGATION_LAMBDA_HPP */
