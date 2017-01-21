#ifndef UPDATE_LAMBDA_HPP
#define UPDATE_LAMBDA_HPP

#include <ros/time.h>
#include <memory>
#include <functional>
#include "particle_set.hpp"

namespace muse_amcl {
class UpdateLambda {
public:
    typedef std::shared_ptr<UpdateLambda> Ptr;

    struct Less {
        bool operator()( const UpdateLambda& lhs,
                         const UpdateLambda& rhs ) const
        {
            return lhs.stamp() > rhs.stamp();
        }
    };

    UpdateLambda(std::function<double (ParticleSet::Weights set)> lambda,
                 const ros::Time & stamp) :
        lambda_(lambda),
        stamp_(stamp)
    {
    }

    inline double operator ()(ParticleSet::Weights set)
    {
        return lambda_(set);
    }

    inline const ros::Time & stamp() const
    {
        return stamp_;
    }

private:
    std::function<double (ParticleSet::Weights set)>  lambda_;
    ros::Time                                                stamp_;

};
}

#endif /* UPDATE_LAMBDA_HPP */
