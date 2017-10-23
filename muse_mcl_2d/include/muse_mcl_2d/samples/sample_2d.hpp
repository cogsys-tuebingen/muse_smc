#ifndef SAMPLE_2D_HPP
#define SAMPLE_2D_HPP

#include <memory>

#include <cslibs_math_2d/point_2d.hpp>
#include <cslibs_math_2d/pose_2d.hpp>
#include <cslibs_math_2d/covariance_2d.hpp>

#include <memory>

namespace muse_mcl_2d {
struct Sample2D {
public:
    using Ptr                       = std::shared_ptr<Sample2D>;
    using allocator_t               = std::allocator<Sample2D>;
    using state_t                   = muse_mcl_math_2d::Pose2D;

    double       weight;
    state_t      state;

    inline Sample2D() :
        weight(0.0),
        state(state_t(0.0,0.0))
    {
    }

    inline Sample2D(const Sample2D &other) :
        weight(other.weight),
        state(other.state)
    {
    }

    inline Sample2D(Sample2D &&other) :
        weight(other.weight),
        state(other.state)
    {
    }


    inline Sample2D& operator = (const Sample2D &other)
    {
        if(&other != this) {
            weight = other.weight;
            state  = other.state;
        }
        return *this;
    }

    inline Sample2D& operator = (Sample2D &&other)
    {
        if(&other != this) {
            weight = other.weight;
            state  = std::move(other.state);
        }
        return *this;
    }
} __attribute__ ((aligned (128)));
}


#endif // SAMPLE_2D_HPP
