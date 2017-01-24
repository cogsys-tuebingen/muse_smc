#include <gtest/gtest.h>
#include <ros/ros.h>
#include <tf/tf.h>

#include <muse_amcl/math/distribution.hpp>

#include "test_map.hpp"
#include "test_map_provider.hpp"
#include "test_uniform_primary_map_2d.hpp"

using namespace muse_amcl;


TEST(TestMuseAMCLCorePlugins, testSamplingUniformPrimary2D)
{
    TFProvider::Ptr tf_provider(new TFProvider);

    std::vector<std::string> frames;
    while(frames.size() != 3) {
        ros::Duration(0.5).sleep();
        tf_provider->getFrameStrings(frames);
    }

    /// setup the sampler
    ros::NodeHandle nh_private("~");
    TestUniformPrimary2D  uniform2d;
    std::map<std::string, muse_amcl::MapProvider::Ptr> map_providers;

    /// prepare the maps
    TestMap::Ptr map0(new TestMap("map0", math::Point(-1, -1), math::Point(1,1)));
    TestMap::Ptr map1(new TestMap("map1", math::Point(-1, -1), math::Point(1,1)));
    map_providers["map0"] = TestMapProvider::Ptr(new TestMapProvider("map0", map0));
    map_providers["map1"] = TestMapProvider::Ptr(new TestMapProvider("map1", map1));

    /// setup
    uniform2d.setup("particle_filter/uniform_pose_generation",
                    nh_private,
                    map_providers,
                    tf_provider);

    EXPECT_EQ(2, uniform2d.getRandomSeed());
    EXPECT_EQ("particle_filter/uniform_pose_generation", uniform2d.getName());
    EXPECT_EQ(4223, uniform2d.getSampleSize());
    EXPECT_EQ(ros::Duration(11.0), uniform2d.getSamplingTimeout());
    EXPECT_EQ(ros::Duration(0.2),  uniform2d.getTFTimeout());
    EXPECT_EQ(tf_provider,         uniform2d.getTFProvider());


    std::vector<MapProvider::Ptr> list_of_secondary_maps = uniform2d.getSecondaryMapProviders();
    EXPECT_EQ(1, list_of_secondary_maps.size());
    EXPECT_EQ("map0", list_of_secondary_maps.front()->getName());
    EXPECT_FALSE((!uniform2d.getPrimaryMapProvider()));
    EXPECT_EQ("map1", uniform2d.getPrimaryMapProvider()->getName());

    /// fire up the tests
    Indexation       indexation({0.1, 0.1, M_PI / 18.0});
    ParticleSet      particle_set("world", 400, indexation);
    uniform2d.apply(particle_set);


    math::statistic::Distribution<3> distribution;
    const ParticleSet::Particles &particles = particle_set.getSamples();
    for(auto &particle : particles) {
        Eigen::Vector3d pose = particle.pose_.eigen3D();
        distribution.add(pose);
    }

    Eigen::Vector3d mu(0.5, 0.5, 0.0);
    Eigen::Vector3d mu_est = distribution.getMean();
    Eigen::Matrix3d sigma_est = distribution.getCovariance();

    EXPECT_EQ(4223, particles.size());
    EXPECT_NEAR(mu(0), mu_est(0), 1e-1);
    EXPECT_NEAR(mu(1), mu_est(1), 1e-1);
    EXPECT_NEAR(mu(2), mu_est(2), 1e-1);

    /// points are expected to be between 0 and 1 in x and y direction
    /// therefore we expect a variance of V(X) = (1 - 0)^2 / 12.0 for theese directions
    /// angular covariance does not need to be evaluated with von mises distribution since
    /// we can assure that values are from interval -pi to pi. Here we expect (2 * M_PI)^2 / 12.0 as variance.
    EXPECT_NEAR((1. / 12.0), sigma_est(0,0), 1e-1);
    EXPECT_NEAR((1. / 12.0), sigma_est(1,1), 1e-1);
    EXPECT_NEAR((4 * M_PI * M_PI / 12.0),  sigma_est(2,2), 1e-1);
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "muse_amcl_core_plugins_test_node_sampling_uniform_primary_map_2d");   /// It's essential to set the right name here !
    ros::Time::init();

    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
