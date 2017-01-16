#include <muse_amcl/math/distribution.hpp>

#include <gtest/gtest.h>
#include <ros/ros.h>

#include "test_distribution.hpp"

muse_amcl::TestDistribution<3> test_distribution_a;
muse_amcl::TestDistribution<3> test_distribution_b;

namespace mms = muse_amcl::math::statistic;

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "muse_amcl_test_node_clustering");
    ros::NodeHandle nh_private("~");

    std::string test_distribution_a_path;
    std::string test_distribution_b_path;
    if(!nh_private.getParam("test_distribution_a", test_distribution_a_path)) {
        std::cerr << "[TestNodeClustering]: Cannot load test_distribution_a!" << std::endl;
        std::cout << test_distribution_a_path << std::endl;
        return -1;
    }
    if(!nh_private.getParam("test_distribution_b", test_distribution_b_path)) {
        std::cerr << "[TestNodeClustering]: Cannot load test_distribution_b!" << std::endl;
        return -1;
    }

    test_distribution_a.read(test_distribution_a_path);
    test_distribution_b.read(test_distribution_b_path);

    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}


