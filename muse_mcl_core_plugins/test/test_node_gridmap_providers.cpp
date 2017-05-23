#include <gtest/gtest.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include <iostream>

#include <atomic>
#include <thread>
#include <muse_mcl/math/distribution.hpp>
#include <muse_mcl/plugins/plugin_loader.hpp>
#include <muse_mcl/data_sources/map_provider.hpp>

using namespace muse_mcl;

std::map<std::string, MapProvider::Ptr>  map_providers;
PluginLoader *loader;

TEST(TestMuseMCLCorePlugins, testLoadMapProviders)
{
    ros::NodeHandle nh_private("~");

    loader = new PluginLoader(nh_private);
    loader->load<MapProvider, ros::NodeHandle&>(map_providers, nh_private);
    EXPECT_EQ(6, map_providers.size());
    EXPECT_TRUE(map_providers.find("binary_gridmap") != map_providers.end());
    EXPECT_TRUE(map_providers.find("distance_gridmap") != map_providers.end());
    EXPECT_TRUE(map_providers.find("probability_gridmap") != map_providers.end());
    EXPECT_TRUE(map_providers.find("binary_gridmap_service") != map_providers.end());
    EXPECT_TRUE(map_providers.find("distance_gridmap_service") != map_providers.end());
    EXPECT_TRUE(map_providers.find("probability_gridmap_service") != map_providers.end());
}

TEST(TestMuseMCLCorePlugins, testGetMapsTopic)
{
    /// binary gridmap provider
    MapProvider::Ptr map_provider = map_providers["binary_gridmap"];
    Map::ConstPtr map;
    std::size_t tries = 0;
//    while(!map) {
//        ++tries;
//        map = map_provider->getMap();
//        ros::Rate(10).sleep();
//    }
//    EXPECT_EQ(1, tries);

    /// distance gridmap provider
    map_provider = map_providers["distance_gridmap"];
    tries = 0;
    map.reset();
    while(!map) {
        ++tries;
        map = map_provider->getMap();
        ros::Rate(10).sleep();
    }
    EXPECT_EQ(1, tries);

//    /// probability gridmap provider
//    map_provider = map_providers["probability_gridmap"];
//    tries = 0;
//    map.reset();
//    while(!map) {
//        ++tries;
//        map = map_provider->getMap();
//        ros::Rate(10).sleep();
//    }
//    EXPECT_EQ(1, tries);

//    /// binary gridmap service
//    map_provider = map_providers["binary_gridmap_service"];
//    tries = 0;
//    map.reset();
//    while(!map) {
//        ++tries;
//        map = map_provider->getMap();
//        ros::Rate(10).sleep();
//    }
//    EXPECT_EQ(1, tries);

//    /// distance gridmap service
//    map_provider = map_providers["distance_gridmap_service"];
//    tries = 0;
//    map.reset();
//    while(!map) {
//        ++tries;
//        map = map_provider->getMap();
//        ros::Rate(10).sleep();
//    }
//    EXPECT_EQ(1, tries);

//    /// probability gridmap service
//    map_provider = map_providers["probability_gridmap_service"];
//    tries = 0;
//    map.reset();
//    while(!map) {
//        ++tries;
//        map = map_provider->getMap();
//        ros::Rate(10).sleep();
//    }
    EXPECT_EQ(1, tries);

}


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "muse_mcl_core_plugins_test_node_gridmap_providers");   /// It's essential to set the right name here !
    ros::Time::init();

    std::atomic_bool stop_ros_loop(false);
    auto rosloop = [&stop_ros_loop]() {
        while(!stop_ros_loop) {
            ros::spinOnce();
            ros::Rate(30).sleep();
        }
    };

    std::thread roslooper(rosloop);
    roslooper.detach();
    testing::InitGoogleTest(&argc, argv);
    int test_result = RUN_ALL_TESTS();
    stop_ros_loop = true;
    if(roslooper.joinable())
        roslooper.join();
    return test_result;
}
