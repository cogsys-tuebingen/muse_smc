#include <muse_mcl/math/distribution.hpp>

#include <gtest/gtest.h>
#include <ros/ros.h>

#include "test_distribution.hpp"

#include <cslibs_indexed_storage/storage.hpp>
#include <cslibs_indexed_storage/backend/kdtree/kdtree_buffered.hpp>
#include <cslibs_indexed_storage/backend/array/array.hpp>
#include <cslibs_indexed_storage/backend/kdtree/kdtree.hpp>

#include <muse_mcl/particle_filter/indexation.hpp>
#include <muse_mcl/particle_filter/clustering_impl.hpp>
#include <muse_mcl/particle_filter/particle.hpp>

namespace cis = cslibs_indexed_storage;

namespace muse_mcl {
namespace clustering {

template<typename Storage>
inline void create(const Indexation &indexation,
                   const std::vector<Particle> &particles,
                   Storage &store)
{
    for(const auto &sample : particles)
    {
        store.insert(indexation.create(sample), Data(sample));
    }
}

template<typename Storage>
void cluster(Storage    &store,
             ClusteringImpl &clusters)
{
    cis::operations::clustering::Clustering<Storage> co(store);
    co.cluster(clusters);
}

using KDTreeBuffered = cis::Storage<Data, Indexation::IndexType::Base, cis::backend::kdtree::KDTreeBuffered>;
using KDTree = cis::Storage<Data, Indexation::IndexType::Base, cis::backend::kdtree::KDTree>;
using Array  = cis::Storage<Data, Indexation::IndexType::Base, cis::backend::array::Array>;

using KDTreeBufferedPtr = std::shared_ptr<KDTreeBuffered>;
using KDTreePtr = std::shared_ptr<KDTree>;
using ArrayPtr = std::shared_ptr<Array>;

}
}


muse_mcl::TestDistribution<3> test_distribution_a;
muse_mcl::TestDistribution<3> test_distribution_b;
std::vector<muse_mcl::Particle> test_samples;


muse_mcl::clustering::KDTreeBuffered  kdtree_buffered;
muse_mcl::clustering::KDTree          kdtree;
muse_mcl::clustering::Array           array;
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

TEST(TestMuseAMCL, createStorage)
{
    /// bins of 10 by 10 cm and and angular resolution of 10 degrees
    /// unbuffered kdtree implementation
    muse_mcl::Indexation index({0.1, 0.1, 1. / 18. * M_PI});

    using Index = muse_mcl::Indexation::IndexType;
    using Size  = std::array<std::size_t, 3>;

    Index exp_min_index      = {{-2, -3, -11}};
    Index exp_max_index      = {{51, 52,  11}};
    Size  exp_size           = {{54, 56, 23}};

    Index min_index = index.create({min(0), min(1), min(2)});
    Index max_index = index.create({max(0), max(1), max(2)});
    Size  size = index.size({{min_index[0], min_index[1], min_index[2]}},
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

    /// in active use, the array does not have to be reinitialized, only the
    /// origin
    EXPECT_NO_FATAL_FAILURE(array.set<cis::option::tags::array_size>(size[0], size[1], size[2]));
    array.set<cis::option::tags::array_offset>(min_index[0], min_index[1], min_index[2]);
    kdtree_buffered.set<cis::option::tags::node_allocator_chunk_size>(2 * test_samples.size() + 1);
    EXPECT_NO_FATAL_FAILURE(muse_mcl::clustering::create(index, test_samples, kdtree_buffered));
    EXPECT_NO_FATAL_FAILURE(muse_mcl::clustering::create(index, test_samples, kdtree));
    EXPECT_NO_FATAL_FAILURE(muse_mcl::clustering::create(index, test_samples, array));
}

TEST(TestMuseAMCL, testClustering)
{
    muse_mcl::clustering::ClusteringImpl clusters_kdtree_buffered;
    muse_mcl::clustering::ClusteringImpl clusters_kdtree;
    muse_mcl::clustering::ClusteringImpl clusters_array;
    EXPECT_NO_FATAL_FAILURE(muse_mcl::clustering::cluster(kdtree_buffered, clusters_kdtree_buffered));
    EXPECT_NO_FATAL_FAILURE(muse_mcl::clustering::cluster(kdtree, clusters_kdtree));
    EXPECT_NO_FATAL_FAILURE(muse_mcl::clustering::cluster(array,  clusters_array));

    EXPECT_EQ(2, clusters_kdtree_buffered.clusters_.size());
    EXPECT_EQ(2, clusters_kdtree.clusters_.size());
    EXPECT_EQ(2, clusters_array.clusters_.size());
    EXPECT_EQ(1, clusters_kdtree_buffered.current_cluster_);
    EXPECT_EQ(1, clusters_kdtree.current_cluster_);
    EXPECT_EQ(1, clusters_array.current_cluster_);

    /// kdtree buffered :
    {
        mms::Distribution<3> distribution_kdtree_buffered_a;
        for(const muse_mcl::Particle *p : clusters_kdtree_buffered.clusters_[0]) {
            distribution_kdtree_buffered_a.add(p->pose_.getEigen3D());
        }
        mms::Distribution<3> distribution_kdtree_buffered_b;
        for(const muse_mcl::Particle *p : clusters_kdtree_buffered.clusters_[1]) {
            distribution_kdtree_buffered_b.add(p->pose_.getEigen3D());
        }

        Eigen::Vector3d mean_a = distribution_kdtree_buffered_a.getMean();
        Eigen::Matrix3d covariance_a = distribution_kdtree_buffered_a.getCovariance();
        Eigen::Vector3d mean_b = distribution_kdtree_buffered_b.getMean();
        Eigen::Matrix3d covariance_b = distribution_kdtree_buffered_b.getCovariance();

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
    /// kdtree :
    {
        mms::Distribution<3> distribution_kdtree_a;
        for(const muse_mcl::Particle *p : clusters_kdtree.clusters_[0]) {
            distribution_kdtree_a.add(p->pose_.getEigen3D());
        }
        mms::Distribution<3> distribution_kdtree_b;
        for(const muse_mcl::Particle *p : clusters_kdtree.clusters_[1]) {
            distribution_kdtree_b.add(p->pose_.getEigen3D());
        }

        Eigen::Vector3d mean_a = distribution_kdtree_a.getMean();
        Eigen::Matrix3d covariance_a = distribution_kdtree_a.getCovariance();
        Eigen::Vector3d mean_b = distribution_kdtree_b.getMean();
        Eigen::Matrix3d covariance_b = distribution_kdtree_b.getCovariance();

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
    /// array :
    {
        mms::Distribution<3> distribution_array_a;
        for(const muse_mcl::Particle *p : clusters_array.clusters_[0]) {
            distribution_array_a.add(p->pose_.getEigen3D());
        }
        mms::Distribution<3> distribution_array_b;
        for(const muse_mcl::Particle *p : clusters_array.clusters_[1]) {
            distribution_array_b.add(p->pose_.getEigen3D());
        }

        Eigen::Vector3d mean_a = distribution_array_a.getMean();
        Eigen::Matrix3d covariance_a = distribution_array_a.getCovariance();
        Eigen::Vector3d mean_b = distribution_array_b.getMean();
        Eigen::Matrix3d covariance_b = distribution_array_b.getCovariance();

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


