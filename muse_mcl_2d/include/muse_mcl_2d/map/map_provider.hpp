#ifndef MAP_PROVIDER_HPP
#define MAP_PROVIDER_HPP

#include <ros/ros.h>

#include <muse_smc/state_space/state_space_provider.hpp>
#include <muse_mcl_2d/samples/sample_2d.hpp>

namespace muse_mcl_2d {
class MapProvider : public muse_smc::StateSpaceProvider<Sample2D>
{
public:
    inline virtual void setup(ros::NodeHandle &nh) = 0;
};
}


#endif // MAP_PROVIDER_HPP
