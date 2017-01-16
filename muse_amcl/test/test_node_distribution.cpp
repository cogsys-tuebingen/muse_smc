#include <muse_amcl/math/distribution.hpp>

#include <gtest/gtest.h>
#include <ros/ros.h>

#include "test_distribution.hpp"

muse_amcl::TestDistribution test_distribution_200;
muse_amcl::TestDistribution test_distribution_500;
muse_amcl::TestDistribution test_distribution_5000;


namespace mms = muse_amcl::math::statistic;

TEST(TestMuseAMCL, testDistributionInsertion)
{
    mms::Distribution<2> distribution;
    EXPECT_EQ(0, distribution.getN());
    for(std::size_t i = 0 ; i < test_distribution_200.data.size() ; ++i) {
        EXPECT_EQ(i, distribution.getN());
        distribution.add(test_distribution_200.data[i]);
    }
    EXPECT_EQ(test_distribution_200.data.size(), distribution.getN());

    distribution.reset();
    EXPECT_EQ(0, distribution.getN());
    for(std::size_t i = 0 ; i < test_distribution_500.data.size() ; ++i) {
        EXPECT_EQ(i, distribution.getN());
        distribution.add(test_distribution_500.data[i]);
    }
    EXPECT_EQ(test_distribution_500.data.size(), distribution.getN());

    distribution.reset();
    EXPECT_EQ(0, distribution.getN());
    for(std::size_t i = 0 ; i < test_distribution_5000.data.size() ; ++i) {
        EXPECT_EQ(i, distribution.getN());
        distribution.add(test_distribution_5000.data[i]);
    }
    EXPECT_EQ(test_distribution_5000.data.size(), distribution.getN());

}

TEST(TestMuseAMCL, testDistributionMean)
{
    const double tolerance = 1e-3;

    mms::Distribution<2> distribution;
    Eigen::Vector2d mean;
    for(std::size_t i = 0 ; i < test_distribution_200.data.size() ; ++i) {
        distribution.add(test_distribution_200.data[i]);
    }

    mean = distribution.getMean();

    EXPECT_NEAR(test_distribution_200.mean(0), mean(0), tolerance);
    EXPECT_NEAR(test_distribution_200.mean(1), mean(1), tolerance);

    distribution.reset();
    for(std::size_t i = 0 ; i < test_distribution_500.data.size() ; ++i) {
        distribution.add(test_distribution_500.data[i]);
    }

    mean = distribution.getMean();
    EXPECT_NEAR(test_distribution_500.mean(0), mean(0), tolerance);
    EXPECT_NEAR(test_distribution_500.mean(1), mean(1), tolerance);

    distribution.reset();
    for(std::size_t i = 0 ; i < test_distribution_5000.data.size() ; ++i) {
        distribution.add(test_distribution_5000.data[i]);
    }

    mean = distribution.getMean();
    EXPECT_NEAR(test_distribution_5000.mean(0), mean(0), tolerance);
    EXPECT_NEAR(test_distribution_5000.mean(1), mean(1), tolerance);
}

TEST(TestMuseAMCL, testDistributionCovariance)
{
    const double tolerance = 1e-3;

    mms::Distribution<2> distribution;
    Eigen::Matrix2d cov;
    for(std::size_t i = 0 ; i < test_distribution_200.data.size() ; ++i) {
        distribution.add(test_distribution_200.data[i]);
    }

    cov = distribution.getCovariance();

    EXPECT_NEAR(test_distribution_200.covariance(0,0), cov(0,0), tolerance);
    EXPECT_NEAR(test_distribution_200.covariance(0,1), cov(0,1), tolerance);
    EXPECT_NEAR(test_distribution_200.covariance(1,0), cov(1,0), tolerance);
    EXPECT_NEAR(test_distribution_200.covariance(1,1), cov(1,1), tolerance);

    distribution.reset();
    for(std::size_t i = 0 ; i < test_distribution_500.data.size() ; ++i) {
        distribution.add(test_distribution_500.data[i]);
    }

    cov = distribution.getCovariance();
    EXPECT_NEAR(test_distribution_500.covariance(0,0), cov(0,0), tolerance);
    EXPECT_NEAR(test_distribution_500.covariance(0,1), cov(0,1), tolerance);
    EXPECT_NEAR(test_distribution_500.covariance(1,0), cov(1,0), tolerance);
    EXPECT_NEAR(test_distribution_500.covariance(1,1), cov(1,1), tolerance);

    distribution.reset();
    for(std::size_t i = 0 ; i < test_distribution_5000.data.size() ; ++i) {
        distribution.add(test_distribution_5000.data[i]);
    }

    cov = distribution.getCovariance();
    EXPECT_NEAR(test_distribution_5000.covariance(0,0), cov(0,0), tolerance);
    EXPECT_NEAR(test_distribution_5000.covariance(0,1), cov(0,1), tolerance);
    EXPECT_NEAR(test_distribution_5000.covariance(1,0), cov(1,0), tolerance);
    EXPECT_NEAR(test_distribution_5000.covariance(1,1), cov(1,1), tolerance);
}

TEST(TestMuseAMCL, testDistributionEigenValues)
{
    const double tolerance = 1e-3;

    mms::Distribution<2> distribution;
    Eigen::Vector2d eigen_values;
    for(std::size_t i = 0 ; i < test_distribution_200.data.size() ; ++i) {
        distribution.add(test_distribution_200.data[i]);
    }

    eigen_values = distribution.getEigenValues();

    EXPECT_NEAR(test_distribution_200.eigen_values(0), eigen_values(0), tolerance);
    EXPECT_NEAR(test_distribution_200.eigen_values(1), eigen_values(1), tolerance);

    distribution.reset();
    for(std::size_t i = 0 ; i < test_distribution_500.data.size() ; ++i) {
        distribution.add(test_distribution_500.data[i]);
    }

    eigen_values = distribution.getEigenValues();
    EXPECT_NEAR(test_distribution_500.eigen_values(0), eigen_values(1), tolerance);
    EXPECT_NEAR(test_distribution_500.eigen_values(1), eigen_values(0), tolerance);

    distribution.reset();
    for(std::size_t i = 0 ; i < test_distribution_5000.data.size() ; ++i) {
        distribution.add(test_distribution_5000.data[i]);
    }

    eigen_values = distribution.getEigenValues();
    EXPECT_NEAR(test_distribution_5000.eigen_values(0), eigen_values(0), tolerance);
    EXPECT_NEAR(test_distribution_5000.eigen_values(1), eigen_values(1), tolerance);
}

TEST(TestMuseAMCL, testDistributionEigenVectors)
{
    auto equals = [] (const Eigen::Vector2d &a,
            const Eigen::Vector2d &b,
            const double eps) {

        Eigen::Matrix2d invert_direction = Eigen::Matrix2d::Identity() * -1;
        Eigen::Vector2d diff_a = a - b;
        Eigen::Vector2d diff_b = a - invert_direction * b;

        return (fabs(diff_a(0)) <= eps && fabs(diff_a(1)) <= eps) ||
               (fabs(diff_b(0)) <= eps && fabs(diff_b(1)) <= eps);
    };

    const double tolerance = 1e-3;

    mms::Distribution<2> distribution;
    Eigen::Matrix2d eigen_vectors;
    for(std::size_t i = 0 ; i < test_distribution_200.data.size() ; ++i) {
        distribution.add(test_distribution_200.data[i]);
    }

    eigen_vectors = distribution.getEigenVectors();

    Eigen::Vector2d exp_a = test_distribution_200.eigen_vectors.col(0);
    Eigen::Vector2d exp_b = test_distribution_200.eigen_vectors.col(1);

    Eigen::Vector2d rec_a = eigen_vectors.col(0);
    Eigen::Vector2d rec_b = eigen_vectors.col(1);
    /// direction and storage order of vectors has not to euqal
    bool condition = (equals(exp_a, rec_a, tolerance) && equals(exp_b, rec_b, tolerance)) ||
                     (equals(exp_a, rec_b, tolerance) && equals(exp_b, rec_a, tolerance));

    EXPECT_TRUE(condition);

    /// 500
    distribution.reset();
    for(std::size_t i = 0 ; i < test_distribution_500.data.size() ; ++i) {
        distribution.add(test_distribution_500.data[i]);
    }

    eigen_vectors = distribution.getEigenVectors();

    exp_a = test_distribution_500.eigen_vectors.col(0);
    exp_b = test_distribution_500.eigen_vectors.col(1);

    rec_a = eigen_vectors.col(0);
    rec_b = eigen_vectors.col(1);
    /// direction and storage order of vectors has not to euqal
    condition = (equals(exp_a, rec_a, tolerance) && equals(exp_b, rec_b, tolerance)) ||
                (equals(exp_a, rec_b, tolerance) && equals(exp_b, rec_a, tolerance));

    EXPECT_TRUE(condition);



    /// 5000
    distribution.reset();
    for(std::size_t i = 0 ; i < test_distribution_5000.data.size() ; ++i) {
        distribution.add(test_distribution_5000.data[i]);
    }

    eigen_vectors = distribution.getEigenVectors();

    exp_a = test_distribution_5000.eigen_vectors.col(0);
    exp_b = test_distribution_5000.eigen_vectors.col(1);

    rec_a = eigen_vectors.col(0);
    rec_b = eigen_vectors.col(1);
    /// direction and storage order of vectors has not to euqal
    condition = (equals(exp_a, rec_a, tolerance) && equals(exp_b, rec_b, tolerance)) ||
                (equals(exp_a, rec_b, tolerance) && equals(exp_b, rec_a, tolerance));

    EXPECT_TRUE(condition);

}

TEST(TestMuseAMCL, testDistributionCopy)
{
    mms::Distribution<2> distribution_a;
    mms::Distribution<2> distribution_b;
    for(std::size_t i = 0 ; i < test_distribution_200.data.size() ; ++i) {
        distribution_a.add(test_distribution_200.data[i]);
    }

    distribution_b = distribution_a;

    EXPECT_TRUE(distribution_a.getMean() == distribution_b.getMean());
    EXPECT_TRUE(distribution_a.getN()    == distribution_b.getN());

    EXPECT_TRUE(distribution_a.getCovariance() == distribution_b.getCovariance());
    EXPECT_TRUE(distribution_a.getInformationMatrix()    == distribution_b.getInformationMatrix());

    EXPECT_TRUE(distribution_a.getEigenValues() == distribution_b.getEigenValues());
    EXPECT_TRUE(distribution_a.getEigenVectors()   == distribution_b.getEigenVectors());
}


TEST(TestMuseAMCL, testDistributionAddition)
{
    const double tolerance = 1e-6;

    mms::Distribution<2> distribution_a;
    mms::Distribution<2> distribution_b;
    for(std::size_t i = 0 ; i < test_distribution_200.data.size() ; ++i) {
        distribution_a.add(test_distribution_200.data[i]);
    }

    distribution_b  = distribution_a;
    distribution_b += distribution_a;

    for(std::size_t i = 0 ; i < test_distribution_200.data.size() ; ++i) {
        distribution_a.add(test_distribution_200.data[i]);
    }

    std::cout << (distribution_a.getMean() - distribution_b.getMean()).norm() << std::endl;

    EXPECT_NEAR((distribution_a.getMean() - distribution_b.getMean()).norm(), 0.0, tolerance);
    EXPECT_TRUE(distribution_a.getN() == distribution_b.getN());

    EXPECT_NEAR((distribution_a.getCovariance() - distribution_b.getCovariance()).norm(), 0.0, tolerance);
    EXPECT_NEAR((distribution_a.getInformationMatrix() - distribution_b.getInformationMatrix()).norm(), 0.0, tolerance);

    EXPECT_NEAR((distribution_a.getEigenValues() - distribution_b.getEigenValues()).norm(), 0.0, tolerance);
    EXPECT_NEAR((distribution_a.getEigenVectors() - distribution_b.getEigenVectors()).norm(), 0.0, tolerance);
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "muse_amcl_test_node_distribution");
    ros::NodeHandle nh_private("~");

    std::string test_distribution_200_path;
    std::string test_distribution_500_path;
    std::string test_distribution_5000_path;
    if(!nh_private.getParam("test_distribution_200", test_distribution_200_path)) {
        std::cerr << "[TestNodeDistribution]: Cannot load test_distribution_200!" << std::endl;
        std::cout << test_distribution_200_path << std::endl;
        return -1;
    }
    if(!nh_private.getParam("test_distribution_500", test_distribution_500_path)) {
        std::cerr << "[TestNodeDistribution]: Cannot load test_distribution_500!" << std::endl;
        return -1;
    }
    if(!nh_private.getParam("test_distribution_5000", test_distribution_5000_path)) {
        std::cerr << "[TestNodeDistribution]: Cannot load test_distribution_5000!" << std::endl;
        return -1;
    }

    test_distribution_200.read(test_distribution_200_path);
    test_distribution_500.read(test_distribution_500_path);
    test_distribution_5000.read(test_distribution_5000_path);


    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}


