#include <gtest/gtest.h>
#include <ros/ros.h>
#include <tf/tf.h>

#include <muse_amcl/math/distribution.hpp>

#include "test_map.hpp"
#include "test_map_provider.hpp"
#include "test_normal_2d.hpp"

#include "../src/sampling/normal_2d.h"


using namespace muse_mcl;



TEST(TestMuseAMCLCorePlugins, testNormalSampling2D)
{
    TFProvider::Ptr tf_provider(new TFProvider);

    std::vector<std::string> frames;
    while(frames.size() != 3) {
        ros::Duration(0.5).sleep();
        tf_provider->getFrameStrings(frames);
    }

    /// setup the sampler
    ros::NodeHandle nh_private("~");
    TestNormal2D  normal2d;
    std::map<std::string, muse_mcl::MapProvider::Ptr> map_providers;

    /// prepare the maps
    TestMap::Ptr map0(new TestMap("map0", math::Point(-1, -1), math::Point(1,1)));
    TestMap::Ptr map1(new TestMap("map1", math::Point(-1, -1), math::Point(1,1)));
    map_providers["map0"] = TestMapProvider::Ptr(new TestMapProvider("map0", map0));
    map_providers["map1"] = TestMapProvider::Ptr(new TestMapProvider("map1", map1));

    /// setup
    normal2d.setup("particle_filter/normal_pose_generation",
                    map_providers,
                    tf_provider,
                    nh_private);

    EXPECT_EQ(0, normal2d.getRandomSeed());
    EXPECT_EQ("particle_filter/normal_pose_generation", normal2d.getName());
    EXPECT_EQ(5000, normal2d.getSampleSize());
    EXPECT_EQ(ros::Duration(10.0), normal2d.getSamplingTimeout());
    EXPECT_EQ(ros::Duration(0.1),  normal2d.getTFTimeout());
    EXPECT_EQ(tf_provider,         normal2d.getTFProvider());
    std::vector<MapProvider::Ptr> list_of_mps = normal2d.getMapProviders();
    EXPECT_EQ(2, list_of_mps.size());
    std::map<std::string, int> counts;
    for(auto m : list_of_mps) {
        ++counts[m->getName()];
    }
    EXPECT_TRUE(counts.find("map0") != counts.end());
    EXPECT_TRUE(counts.find("map1") != counts.end());
    for(auto &e : counts) {
        EXPECT_EQ(1, e.second);
    }

    /// fire up the tests
    Eigen::Vector3d  mu_vec = {0.5, 0.5, 0.0};
    Eigen::Matrix3d  sigma_mat = Eigen::Matrix3d::Identity() * 0.5;
    sigma_mat(2,2) = 0.087 * 0.087; /// around 5 degrees standard deviation

    math::Pose       mu(mu_vec);
    math::Covariance sigma(sigma_mat);
    Indexation       indexation({0.1, 0.1, M_PI / 18.0});
    ParticleSet      particle_set("world", 10, 5000, indexation);

    normal2d.apply(mu, sigma, particle_set);
    math::statistic::Distribution<3> distribution;
    const ParticleSet::Particles &particles = particle_set.getSamples();
    for(auto &particle : particles) {
        Eigen::Vector3d tmp = particle.pose_.getEigen3D();
        distribution.add(tmp);
    }

    Eigen::Vector3d mu_est = distribution.getMean();
    Eigen::Matrix3d sigma_est = distribution.getCovariance();

    EXPECT_EQ(5000, particles.size());
    EXPECT_NEAR(mu.getEigen3D()(0), mu_est(0), 1e-1);
    EXPECT_NEAR(mu.getEigen3D()(1), mu_est(1), 1e-1);
    EXPECT_NEAR(mu.getEigen3D()(2), mu_est(2), 1e-1);

    EXPECT_NEAR(sigma(0,0), sigma_est(0,0), 1e-2);
    EXPECT_NEAR(sigma(1,1), sigma_est(1,1), 1e-2);
    EXPECT_NEAR(sigma(2,2), sigma_est(2,2), 1e-2);

}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "muse_amcl_core_plugins_test_node_sampling_normal_2d");   /// It's essential to set the right name here !
    ros::Time::init();

    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
