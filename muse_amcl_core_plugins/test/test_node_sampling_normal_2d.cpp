#include <gtest/gtest.h>
#include <ros/ros.h>
#include <tf/tf.h>

#include "test_map.hpp"
#include "test_map_provider.hpp"

#include "../src/sampling/normal_2d.h"


using namespace muse_amcl;

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "test_node_sampling_normal_2d");


    /// prepare the maps
    TestMap::Ptr map0(new TestMap("map0", math::Point(-1, -1), math::Point(1,1)));
    TestMap::Ptr map1(new TestMap("map1", math::Point(-1, -1), math::Point(1,1)));

    std::map<std::string, muse_amcl::MapProvider::Ptr> map_providers;
    map_providers["map0"] = TestMapProvider::Ptr(new TestMapProvider("map0", map0));
    map_providers["map1"] = TestMapProvider::Ptr(new TestMapProvider("map1", map1));

    /// setup the sampler
    ros::NodeHandle nh_private("~");
    Normal2D sampler;
    TFProvider::Ptr tf(new TFProvider);
    sampler.setup("normal_pose_generation", nh_private, map_providers, tf);

    /// fire up the tests
    math::Pose       pose;
    math::Covariance cov;
    ParticleSet      particle_set("world", 400);

    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
