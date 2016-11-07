#pragma once

#include <vector>
#include "types/update.hpp"
#include "types/propagation.hpp"
#include "types/data_provider.hpp"
#include "types/map_provider.hpp"

#include <ros/node_handle.h>

namespace muse_amcl
{
class PluginLoader
{
public:
    PluginLoader();

    static void load(ros::NodeHandle &nh_private) { /* get up wrapper classes */}

};
}
