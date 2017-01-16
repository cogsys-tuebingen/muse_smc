#include <muse_amcl/math/distribution.hpp>
#include <muse_amcl/particle_filter/clustering.hpp>

#include <gtest/gtest.h>
#include <ros/ros.h>

#include "test_distribution.hpp"

muse_amcl::TestDistribution<3> test_distribution_a;
muse_amcl::TestDistribution<3> test_distribution_b;
std::vector<muse_amcl::Particle> test_distribution_a_samples;
std::vector<muse_amcl::Particle> test_distribution_b_samples;

muse_amcl::clustering::KDTree  kdtree_a;
muse_amcl::clustering::KDTree  kdtree_b;

namespace mms = muse_amcl::math::statistic;


TEST(TestMuseAMCL, testTestDistributionRead)
{
    EXPECT_EQ(1000, test_distribution_a.data.size());
    EXPECT_EQ(1000, test_distribution_b.data.size());

    const std::vector<double> exp_sigma_a = {0.002568661558998087, -0.00011524049587638333, 0.00031798922495299695, -0.00011524049587638333,
                                             0.002478226554103719, 0.0004279908301365344, 0.00031798922495299695, 0.0004279908301365344,
                                             0.1296106703632887};
    const std::vector<double> exp_mu_a    = {-0.0006855518443336871, -0.001996609940083168, 0.7867056040542918};
    const std::vector<double> exp_sigma_b = {0.0025956395944793207, 6.80699933604188e-05, -0.0012600465807219591, 6.80699933604188e-05,
                                             0.002464316990121675, -0.0016997486445437064, -0.0012600465807219591, -0.0016997486445437064,
                                             0.11690125413810105};
    const std::vector<double> exp_mu_b    = {5.000238242637764, 4.999439346820392, -0.784468535793019};
    std::size_t it = 0;
    for(std::size_t i = 0 ; i < 3 ; ++i) {
        EXPECT_EQ((exp_mu_a[i]), (test_distribution_a.mean(i)));
        EXPECT_EQ((exp_mu_b[i]), (test_distribution_b.mean(i)));
        for(std::size_t j = 0 ; j < 3 ; ++j, ++it) {
            EXPECT_EQ((exp_sigma_a[it]), (test_distribution_a.covariance(i,j)));
            EXPECT_EQ((exp_sigma_b[it]), (test_distribution_b.covariance(i,j)));
        }
    }

    const double weight_a = 1.0 / 1000.0;
    for(auto &s : test_distribution_a.data) {
        muse_amcl::Particle p(s, weight_a);
        test_distribution_a_samples.emplace_back(p);
    }
    const double weight_b = 1.0 / 1000.0;
    for(auto &s : test_distribution_b.data) {
        muse_amcl::Particle p(s, weight_b);
        test_distribution_b_samples.emplace_back(p);
    }
}

TEST(TestMuseAMCL, createStorage)
{
}

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


