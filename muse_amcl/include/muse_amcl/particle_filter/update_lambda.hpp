#ifndef UPDATE_LAMBDA_HPP
#define UPDATE_LAMBDA_HPP

#include <chrono>
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

    UpdateLambda(std::function<double (ParticleSet::WeightIterator set)> lambda,
                 std::chrono::time_point<std::chrono::system_clock>   stamp) :
        lambda_(lambda),
        stamp_(stamp)
    {
    }

    inline double operator ()(ParticleSet::WeightIterator set)
    {
        return lambda_(set);
    }

    inline std::chrono::time_point<std::chrono::system_clock> stamp() const
    {
        return stamp_;
    }

private:
    std::function<double (ParticleSet::WeightIterator set)>  lambda_;
    std::chrono::time_point<std::chrono::system_clock>    stamp_;

};
}

#endif /* UPDATE_LAMBDA_HPP */
