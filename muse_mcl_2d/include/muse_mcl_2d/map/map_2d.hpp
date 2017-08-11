#ifndef MAP_2D_HPP
#define MAP_2D_HPP

#include <ros/ros.h>

#include <muse_smc/state_space/state_space.hpp>
#include <muse_mcl_2d/samples/sample_2d.hpp>
#include <muse_mcl_2d/math/point_2d.hpp>

namespace muse_mcl_2d {
class Map2D : public muse_smc::StateSpace<Sample2D>
{
public:
    using limits_t = std::numeric_limits<double>;

    virtual state_space_boundary_t getMin() const
    {
        return Point2D(limits_t::lowest(),
                       limits_t::lowest());
    }

    virtual state_space_boundary_t getMax() const
    {
        return Point2D(limits_t::max(),
                       limits_t::max());
    }

    virtual state_space_transform_t getOrigin() const
    {
        return Transform2D();
    }

    virtual bool isAvailable() const
    {
        return true;
    }

protected:
    Map2D() = delete;
};
}


#endif // MAP_2D_HPP
