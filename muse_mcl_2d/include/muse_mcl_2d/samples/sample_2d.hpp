#ifndef SAMPLE_2D_HPP
#define SAMPLE_2D_HPP

#include <memory>

#include <muse_mcl_2d/math/point_2d.hpp>
#include <muse_mcl_2d/math/pose_2d.hpp>
#include <muse_mcl_2d/math/covariance_2d.hpp>

namespace muse_mcl_2d {
struct Sample2D {
public:
    using Ptr              = std::shared_ptr<Sample2D>;
    using allocator_t      = std::allocator<Sample2D>;
    using state_t          = Pose2D;
    using state_space_boundary_t = Point2D;
    using transform_t      = Transform2D;
    using covariance_t     = Covariance2D;

    double weight;
    Pose2D state;

    Sample2D() :
        weight(0.0),
        state(Pose2D())
    {
    }

    Sample2D(const Sample2D &other) :
        weight(other.weight),
        state(other.state)
    {
    }

    Sample2D(Sample2D &&other) :
        weight(other.weight),
        state(other.state)
    {
    }


    Sample2D& operator = (const Sample2D &other)
    {
        if(&other != this) {
            weight = other.weight;
            state  = other.state;
        }
        return *this;
    }

    Sample2D& operator = (Sample2D &&other)
    {
        if(&other != this) {
            weight = other.weight;
            state  = std::move(other.state);
        }
        return *this;
    }



};
}


#endif // SAMPLE_2D_HPP
