#include <muse_mcl/math/distribution.hpp>
#include <muse_mcl/particle_filter/indexation.hpp>
#include <muse_mcl/particle_filter/clustering_data.hpp>
#include <muse_mcl/particle_filter/clustering_impl.hpp>
#include <muse_mcl/particle_filter/particle.hpp>
#include <muse_mcl/particle_filter/particle_set.hpp>

#include <gtest/gtest.h>
#include <ros/ros.h>

#include "test_distribution.hpp"

muse_mcl::TestDistribution<3> test_distribution_a;
muse_mcl::TestDistribution<3> test_distribution_b;
std::vector<muse_mcl::Particle> test_samples;

using namespace muse_mcl;
using KDTreeBuffered = cis::Storage<clustering::Data, Indexation::IndexType::Base, cis::backend::kdtree::KDTreeBuffered>;
using Array  = cis::Storage<clustering::Data, Indexation::IndexType::Base, cis::backend::array::Array>;
Eigen::Vector3d max = Eigen::Vector3d::Constant(std::numeric_limits<double>::min());
Eigen::Vector3d min = Eigen::Vector3d::Constant(std::numeric_limits<double>::max());


namespace mms = muse_mcl::math::statistic;


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

    const double weight = 1.0 / 2000.0;
    for(auto &s : test_distribution_a.data) {
        muse_mcl::Particle p(s, weight);
        test_samples.emplace_back(p);
        /// in search for the minimum
        for(std::size_t i = 0 ; i < 3 ; ++i) {
            if(s(i) > max(i))
                max(i) = s(i);
            if(s(i) < min(i))
                min(i) = s(i);
        }
    }
    for(auto &s : test_distribution_b.data) {
        muse_mcl::Particle p(s, weight);
        test_samples.emplace_back(p);
        /// in search for the minimum
        for(std::size_t i = 0 ; i < 3 ; ++i) {
            if(s(i) > max(i))
                max(i) = s(i);
            if(s(i) < min(i))
                min(i) = s(i);
        }
    }

    EXPECT_EQ(test_samples.size(), 2000);
}

TEST(TestMuseAMCL, testParticleSetConstructors)
{
    const std::size_t N = 500000;
    const std::size_t MIN = 10;
    const std::size_t MAX = N;
    muse_mcl::Indexation indexation ({0.1, 0.1, 1./18. * M_PI});
    muse_mcl::ParticleSet particle_set_a("frame", N, indexation);

    const muse_mcl::Indexation::IndexType minimum_index(std::numeric_limits<int>::max());
    const muse_mcl::Indexation::IndexType maximum_index(std::numeric_limits<int>::min());


    EXPECT_EQ(particle_set_a.getSampleSize(), 0);
    EXPECT_EQ(particle_set_a.getSampleSizeMinimum(), N);
    EXPECT_EQ(particle_set_a.getSampleSizeMaximum(), N);
    EXPECT_EQ(particle_set_a.getSampleWeightMaximum(),0.0);
    EXPECT_EQ(particle_set_a.getSampleWeightSum(), 0.0);
    EXPECT_EQ(particle_set_a.getSampleIndexMinimum() ,minimum_index);
    EXPECT_EQ(particle_set_a.getSampleIndexMaximum(), maximum_index);

    particle_set_a =
            muse_mcl::ParticleSet("frame", MIN, MAX, indexation);

    EXPECT_EQ(particle_set_a.getSampleSize(), 0);
    EXPECT_EQ(particle_set_a.getSampleSizeMinimum(), MIN);
    EXPECT_EQ(particle_set_a.getSampleSizeMaximum(), MAX);
    EXPECT_EQ(particle_set_a.getSampleWeightMaximum(), 0.0);
    EXPECT_EQ(particle_set_a.getSampleWeightSum(), 0.0);
    EXPECT_EQ(particle_set_a.getSampleIndexMinimum() ,minimum_index);
    EXPECT_EQ(particle_set_a.getSampleIndexMaximum(), maximum_index);
}

TEST(TestMuseAMCL, fillParticleSetA)
{
    using Index = muse_mcl::Indexation::IndexType;
    using Size  = std::array<std::size_t, 3>;

    muse_mcl::Indexation  indexation ({0.1, 0.1, 1./18. * M_PI});
    muse_mcl::ParticleSet particle_set("world", 10, 2 * test_samples.size(), indexation);
    auto inserter = particle_set.getInsertion();
    for(auto &s : test_samples) {
        if(inserter.canInsert())
            inserter.insert(std::move(s));
    }

    inserter.close();

    Index exp_min_index      = {{-2, -3, -11}};
    Index exp_max_index      = {{51, 52,  11}};
    Size  exp_size           = {{54, 56, 23}};

    Index max_index = particle_set.getSampleIndexMaximum();
    Index min_index = particle_set.getSampleIndexMinimum();
    Size  size = indexation.size({{min_index[0], min_index[1], min_index[2]}},
    {{max_index[0], max_index[1], max_index[2]}});

    EXPECT_EQ(exp_min_index[0], min_index[0]);
    EXPECT_EQ(exp_max_index[0], max_index[0]);
    EXPECT_EQ(exp_min_index[1], min_index[1]);
    EXPECT_EQ(exp_max_index[1], max_index[1]);
    EXPECT_EQ(exp_min_index[2], min_index[2]);
    EXPECT_EQ(exp_max_index[2], max_index[2]);
    EXPECT_EQ(exp_size[0], size[0]);
    EXPECT_EQ(exp_size[1], size[1]);
    EXPECT_EQ(exp_size[2], size[2]);
    EXPECT_EQ(test_samples.size(), particle_set.getSampleSize());
}

TEST(TestMuseAMCL, fillParticleSetB)
{
    using Index = muse_mcl::Indexation::IndexType;
    using Size  = std::array<std::size_t, 3>;

    muse_mcl::Indexation  indexation ({0.1, 0.1, 1./18. * M_PI});
    muse_mcl::ParticleSet particle_set("world", 0, 2 * test_samples.size(), indexation);

    {
        auto i = particle_set.getInsertion();
        for(auto &s : test_samples)  {
            i.insert(std::move(s));
        }
    }
    std::vector<const muse_mcl::Particle*> particles_t_1;
    for(auto &p : particle_set.getSamples()) {
        particles_t_1.emplace_back(&p);
    }

    {
        auto i = particle_set.getInsertion();
        for(auto &s : test_samples)  {
            i.insert(std::move(s));
        }
    }
    std::vector<const muse_mcl::Particle*> particles_t;
    for(auto &p : particle_set.getSamples()) {
        particles_t.emplace_back(&p);
    }

    EXPECT_EQ(particles_t_1.size(), particles_t.size());
    for(std::size_t i = 0 ; i < particles_t.size() ; ++i) {
        EXPECT_TRUE(particles_t[i] != particles_t_1[i]);
    }

    {
        auto i = particle_set.getInsertion();
        for(auto &s : test_samples)  {
            i.insert(std::move(s));
        }
    }

    particles_t.clear();
    for(auto &p : particle_set.getSamples()) {
        particles_t.emplace_back(&p);
    }

    EXPECT_EQ(particles_t_1.size(), particles_t.size());
    for(std::size_t i = 0 ; i < particles_t.size() ; ++i) {
        EXPECT_TRUE(particles_t[i] == particles_t_1[i]);
    }

    Index exp_min_index      = {{-2, -3, -11}};
    Index exp_max_index      = {{51, 52,  11}};
    Size  exp_size           = {{54, 56, 23}};

    Index max_index = particle_set.getSampleIndexMaximum();
    Index min_index = particle_set.getSampleIndexMinimum();
    Size  size = indexation.size({{min_index[0], min_index[1], min_index[2]}},
    {{max_index[0], max_index[1], max_index[2]}});

    EXPECT_EQ(exp_min_index[0], min_index[0]);
    EXPECT_EQ(exp_max_index[0], max_index[0]);
    EXPECT_EQ(exp_min_index[1], min_index[1]);
    EXPECT_EQ(exp_max_index[1], max_index[1]);
    EXPECT_EQ(exp_min_index[2], min_index[2]);
    EXPECT_EQ(exp_max_index[2], max_index[2]);
    EXPECT_EQ(exp_size[0], size[0]);
    EXPECT_EQ(exp_size[1], size[1]);
    EXPECT_EQ(exp_size[2], size[2]);
    EXPECT_EQ(test_samples.size(), particle_set.getSampleSize());
}

TEST(TestMuseAMCL, testWeightIterator)
{
    using Index = muse_mcl::Indexation::IndexType;
    using Size  = std::array<std::size_t, 3>;

    muse_mcl::Indexation  indexation ({0.1, 0.1, 1./18. * M_PI});
    muse_mcl::ParticleSet particle_set("world", 0, 2 * test_samples.size(), indexation);
    auto i = particle_set.getInsertion();
    for(auto &s : test_samples) {
        i.insert(std::move(s));
    }
    i.close();

    EXPECT_EQ(test_samples.size(), particle_set.getSampleSize());

    double s = 0.0;
    auto weights = particle_set.getWeights();
    for(auto &w : weights) {
        w = 1.0;
        s += 1.0;
    }

    EXPECT_EQ(s, particle_set.getSampleWeightSum());
}

TEST(TestMuseAMCL, testPoseIterator)
{
    using Index = muse_mcl::Indexation::IndexType;
    using Size  = std::array<std::size_t, 3>;

    muse_mcl::Indexation  indexation ({0.1, 0.1, 1./18. * M_PI});
    muse_mcl::ParticleSet particle_set("world", 0, 2 * test_samples.size(), indexation);
    auto i = particle_set.getInsertion();
    for(auto &s : test_samples) {
        i.insert(std::move(s));
    }
    i.close();

    auto it = particle_set.getPoses().begin();
    for(auto &s : test_samples) {
        *it = s.pose_;
        ++it;
    }

    Index exp_min_index      = {{-2, -3, -11}};
    Index exp_max_index      = {{51, 52,  11}};
    Size  exp_size           = {{54, 56, 23}};

    Index max_index = particle_set.getSampleIndexMaximum();
    Index min_index = particle_set.getSampleIndexMinimum();
    Size  size = indexation.size({{min_index[0], min_index[1], min_index[2]}},
    {{max_index[0], max_index[1], max_index[2]}});

    EXPECT_EQ(exp_min_index[0], min_index[0]);
    EXPECT_EQ(exp_max_index[0], max_index[0]);
    EXPECT_EQ(exp_min_index[1], min_index[1]);
    EXPECT_EQ(exp_max_index[1], max_index[1]);
    EXPECT_EQ(exp_min_index[2], min_index[2]);
    EXPECT_EQ(exp_max_index[2], max_index[2]);
    EXPECT_EQ(exp_size[0], size[0]);
    EXPECT_EQ(exp_size[1], size[1]);
    EXPECT_EQ(exp_size[2], size[2]);
    EXPECT_EQ(test_samples.size(), particle_set.getSampleSize());
}

TEST(TestMuseAMCL, testClustering)
{
    using Index = muse_mcl::Indexation::IndexType;
    using Size  = std::array<std::size_t, 3>;

    muse_mcl::Indexation  indexation ({0.1, 0.1, 1./18. * M_PI});
    muse_mcl::ParticleSet particle_set("world", 0, 2 * test_samples.size(), indexation);
    auto i = particle_set.getInsertion();
    for(auto &s : test_samples) {
        i.insert(std::move(s));
    }
    i.close();

    EXPECT_NO_FATAL_FAILURE(particle_set.cluster());
    auto clusters = particle_set.getClusters();

    EXPECT_EQ(2, clusters.size());

    mms::Distribution<3> distribution_a;
    for(const muse_mcl::Particle *p : clusters[0]) {
        distribution_a.add(p->pose_.getEigen3D());
    }
    mms::Distribution<3> distribution_b;
    for(const muse_mcl::Particle *p : clusters[1]) {
        distribution_b.add(p->pose_.getEigen3D());
    }

    Eigen::Vector3d mean_a = distribution_a.getMean();
    Eigen::Matrix3d covariance_a = distribution_a.getCovariance();
    Eigen::Vector3d mean_b = distribution_b.getMean();
    Eigen::Matrix3d covariance_b = distribution_b.getCovariance();

    if(std::abs(mean_a(0) - test_distribution_a.mean(0)) < 1e-3 &&
            std::abs(mean_a(1) - test_distribution_a.mean(1)  < 1e-3))
    {
        EXPECT_NEAR(test_distribution_a.mean(0), mean_a(0), 1e-6);
        EXPECT_NEAR(test_distribution_a.mean(1), mean_a(1), 1e-6);
        EXPECT_NEAR(test_distribution_a.mean(2), mean_a(2), 1e-6);

        EXPECT_NEAR(test_distribution_b.mean(0), mean_b(0), 1e-6);
        EXPECT_NEAR(test_distribution_b.mean(1), mean_b(1), 1e-6);
        EXPECT_NEAR(test_distribution_b.mean(2), mean_b(2), 1e-6);

        EXPECT_NEAR(test_distribution_a.covariance(0,0), covariance_a(0,0), 1e-6);
        EXPECT_NEAR(test_distribution_a.covariance(0,1), covariance_a(0,1), 1e-6);
        EXPECT_NEAR(test_distribution_a.covariance(0,2), covariance_a(0,2), 1e-6);
        EXPECT_NEAR(test_distribution_a.covariance(1,0), covariance_a(1,0), 1e-6);
        EXPECT_NEAR(test_distribution_a.covariance(1,1), covariance_a(1,1), 1e-6);
        EXPECT_NEAR(test_distribution_a.covariance(1,2), covariance_a(1,2), 1e-6);
        EXPECT_NEAR(test_distribution_a.covariance(2,0), covariance_a(2,0), 1e-6);
        EXPECT_NEAR(test_distribution_a.covariance(2,1), covariance_a(2,1), 1e-6);
        EXPECT_NEAR(test_distribution_a.covariance(2,2), covariance_a(2,2), 1e-6);

        EXPECT_NEAR(test_distribution_b.covariance(0,0), covariance_b(0,0), 1e-6);
        EXPECT_NEAR(test_distribution_b.covariance(0,1), covariance_b(0,1), 1e-6);
        EXPECT_NEAR(test_distribution_b.covariance(0,2), covariance_b(0,2), 1e-6);
        EXPECT_NEAR(test_distribution_b.covariance(1,0), covariance_b(1,0), 1e-6);
        EXPECT_NEAR(test_distribution_b.covariance(1,1), covariance_b(1,1), 1e-6);
        EXPECT_NEAR(test_distribution_b.covariance(1,2), covariance_b(1,2), 1e-6);
        EXPECT_NEAR(test_distribution_b.covariance(2,0), covariance_b(2,0), 1e-6);
        EXPECT_NEAR(test_distribution_b.covariance(2,1), covariance_b(2,1), 1e-6);
        EXPECT_NEAR(test_distribution_b.covariance(2,2), covariance_b(2,2), 1e-6);


    } else {
        EXPECT_NEAR(test_distribution_a.mean(0), mean_b(0), 1e-6);
        EXPECT_NEAR(test_distribution_a.mean(1), mean_b(1), 1e-6);
        EXPECT_NEAR(test_distribution_a.mean(2), mean_b(2), 1e-6);

        EXPECT_NEAR(test_distribution_b.mean(0), mean_a(0), 1e-6);
        EXPECT_NEAR(test_distribution_b.mean(1), mean_a(1), 1e-6);
        EXPECT_NEAR(test_distribution_b.mean(2), mean_a(2), 1e-6);

        EXPECT_NEAR(test_distribution_a.covariance(0,0), covariance_b(0,0), 1e-6);
        EXPECT_NEAR(test_distribution_a.covariance(0,1), covariance_b(0,1), 1e-6);
        EXPECT_NEAR(test_distribution_a.covariance(0,2), covariance_b(0,2), 1e-6);
        EXPECT_NEAR(test_distribution_a.covariance(1,0), covariance_b(1,0), 1e-6);
        EXPECT_NEAR(test_distribution_a.covariance(1,1), covariance_b(1,1), 1e-6);
        EXPECT_NEAR(test_distribution_a.covariance(1,2), covariance_b(1,2), 1e-6);
        EXPECT_NEAR(test_distribution_a.covariance(2,0), covariance_b(2,0), 1e-6);
        EXPECT_NEAR(test_distribution_a.covariance(2,1), covariance_b(2,1), 1e-6);
        EXPECT_NEAR(test_distribution_a.covariance(2,2), covariance_b(2,2), 1e-6);

        EXPECT_NEAR(test_distribution_b.covariance(0,0), covariance_a(0,0), 1e-6);
        EXPECT_NEAR(test_distribution_b.covariance(0,1), covariance_a(0,1), 1e-6);
        EXPECT_NEAR(test_distribution_b.covariance(0,2), covariance_a(0,2), 1e-6);
        EXPECT_NEAR(test_distribution_b.covariance(1,0), covariance_a(1,0), 1e-6);
        EXPECT_NEAR(test_distribution_b.covariance(1,1), covariance_a(1,1), 1e-6);
        EXPECT_NEAR(test_distribution_b.covariance(1,2), covariance_a(1,2), 1e-6);
        EXPECT_NEAR(test_distribution_b.covariance(2,0), covariance_a(2,0), 1e-6);
        EXPECT_NEAR(test_distribution_b.covariance(2,1), covariance_a(2,1), 1e-6);
        EXPECT_NEAR(test_distribution_b.covariance(2,2), covariance_a(2,2), 1e-6);
    }
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "muse_amcl_test_node_particle_set");
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


