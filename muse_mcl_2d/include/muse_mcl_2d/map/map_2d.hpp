#ifndef MAP_2D_HPP
#define MAP_2D_HPP

#include <ros/ros.h>

#include <muse_smc/state_space/state_space.hpp>

#include <muse_mcl_2d/samples/sample_2d.hpp>
#include <cslibs_math_2d/types/point.hpp>
#include <muse_mcl_2d/state_space/state_space_description_2d.hpp>

namespace muse_mcl_2d {
class Map2D : public muse_smc::StateSpace<StateSpaceDescription2D>
{
public:
    using limits_t = std::numeric_limits<double>;

    inline Map2D(const std::string &frame) :
        muse_smc::StateSpace<StateSpaceDescription2D>(frame)
    {
    }

    virtual state_space_boundary_t getMin() const
    {
        return cslibs_math_2d::Point2d(limits_t::lowest(),
                                         limits_t::lowest());
    }

    virtual state_space_boundary_t getMax() const
    {
        return cslibs_math_2d::Point2d(limits_t::max(),
                                         limits_t::max());
    }

    virtual state_space_transform_t getOrigin() const
    {
        return cslibs_math_2d::Transform2d::identity();
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
