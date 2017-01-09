#include <muse_amcl/math/distribution.hpp>

#include "finite_example_distributions.hpp"

#include <gtest/gtest.h>

#include <fstream>

namespace mms = muse_amcl::math::statistic;

using DataPoint = std::array<double, 2>;
using Distribution200 = example_distributions::Distribution200;
using Distribution500 = example_distributions::Distribution500;
using Distribution5000 = example_distributions::Distribution5000;

TEST(TestMuseAMCL, testDistributionInsertion)
{
    mms::Distribution<2> distribution;
    EXPECT_EQ(0, distribution.getN());
    for(std::size_t i = 0 ; i < Distribution200::size ; ++i) {
        EXPECT_EQ(i, distribution.getN());
        distribution.add({Distribution200::data[i * 2 + 0],
                          Distribution200::data[i * 2 + 1]});
    }
    EXPECT_EQ(Distribution200::size, distribution.getN());

    distribution.reset();
    EXPECT_EQ(0, distribution.getN());
    for(std::size_t i = 0 ; i < Distribution500::size ; ++i) {
        EXPECT_EQ(i, distribution.getN());
        distribution.add({Distribution500::data[i * 2 + 0],
                          Distribution500::data[i * 2 + 1]});
    }
    EXPECT_EQ(Distribution500::size, distribution.getN());

    distribution.reset();
    EXPECT_EQ(0, distribution.getN());
    for(std::size_t i = 0 ; i < Distribution5000::size ; ++i) {
        EXPECT_EQ(i, distribution.getN());
        distribution.add({Distribution5000::data[i * 2 + 0],
                          Distribution5000::data[i * 2 + 1]});
    }
    EXPECT_EQ(Distribution5000::size, distribution.getN());

}

TEST(TestMuseAMCL, testDistributionMean)
{
    const double tolerance = 1e-3;

    mms::Distribution<2> distribution;
    Eigen::Vector2d mean;
    for(std::size_t i = 0 ; i < Distribution200::size ; ++i) {
        distribution.add({Distribution200::data[i * 2 + 0],
                          Distribution200::data[i * 2 + 1]});
    }

    mean = distribution.getMean();

    EXPECT_NEAR(Distribution200::mean[0], mean(0), tolerance);
    EXPECT_NEAR(Distribution200::mean[1], mean(1), tolerance);

    distribution.reset();
    for(std::size_t i = 0 ; i < Distribution500::size ; ++i) {
        distribution.add({Distribution500::data[i * 2 + 0],
                          Distribution500::data[i * 2 + 1]});
    }

    mean = distribution.getMean();
    EXPECT_NEAR(Distribution500::mean[0], mean(0), tolerance);
    EXPECT_NEAR(Distribution500::mean[1], mean(1), tolerance);

    distribution.reset();
    for(std::size_t i = 0 ; i < Distribution5000::size ; ++i) {
        distribution.add({Distribution5000::data[i * 2 + 0],
                          Distribution5000::data[i * 2 + 1]});
    }

    mean = distribution.getMean();
    EXPECT_NEAR(Distribution5000::mean[0], mean(0), tolerance);
    EXPECT_NEAR(Distribution5000::mean[1], mean(1), tolerance);
}

TEST(TestMuseAMCL, testDistributionCovariance)
{
    const double tolerance = 1e-3;

    mms::Distribution<2> distribution;
    Eigen::Matrix2d cov;
    for(std::size_t i = 0 ; i < Distribution200::size ; ++i) {
        distribution.add({Distribution200::data[i * 2 + 0],
                          Distribution200::data[i * 2 + 1]});
    }

    cov = distribution.getCovariance();

    EXPECT_NEAR(Distribution200::cov[0], cov(0,0), tolerance);
    EXPECT_NEAR(Distribution200::cov[1], cov(0,1), tolerance);
    EXPECT_NEAR(Distribution200::cov[2], cov(1,0), tolerance);
    EXPECT_NEAR(Distribution200::cov[3], cov(1,1), tolerance);

    distribution.reset();
    for(std::size_t i = 0 ; i < Distribution500::size ; ++i) {
        distribution.add({Distribution500::data[i * 2 + 0],
                          Distribution500::data[i * 2 + 1]});
    }

    cov = distribution.getCovariance();
    EXPECT_NEAR(Distribution500::cov[0], cov(0,0), tolerance);
    EXPECT_NEAR(Distribution500::cov[1], cov(0,1), tolerance);
    EXPECT_NEAR(Distribution500::cov[2], cov(1,0), tolerance);
    EXPECT_NEAR(Distribution500::cov[3], cov(1,1), tolerance);

    distribution.reset();
    for(std::size_t i = 0 ; i < Distribution5000::size ; ++i) {
        distribution.add({Distribution5000::data[i * 2 + 0],
                          Distribution5000::data[i * 2 + 1]});
    }

    cov = distribution.getCovariance();
    EXPECT_NEAR(Distribution5000::cov[0], cov(0,0), tolerance);
    EXPECT_NEAR(Distribution5000::cov[1], cov(0,1), tolerance);
    EXPECT_NEAR(Distribution5000::cov[2], cov(1,0), tolerance);
    EXPECT_NEAR(Distribution5000::cov[3], cov(1,1), tolerance);
}

TEST(TestMuseAMCL, testDistributionEigenValues)
{
    const double tolerance = 1e-3;

    mms::Distribution<2> distribution;
    Eigen::Vector2d eigen_values;
    for(std::size_t i = 0 ; i < Distribution200::size ; ++i) {
        distribution.add({Distribution200::data[i * 2 + 0],
                          Distribution200::data[i * 2 + 1]});
    }

    eigen_values = distribution.getEigenValues();

    EXPECT_NEAR(Distribution200::eigen_values[0], eigen_values(0), tolerance);
    EXPECT_NEAR(Distribution200::eigen_values[1], eigen_values(1), tolerance);

    distribution.reset();
    for(std::size_t i = 0 ; i < Distribution500::size ; ++i) {
        distribution.add({Distribution500::data[i * 2 + 0],
                          Distribution500::data[i * 2 + 1]});
    }

    eigen_values = distribution.getEigenValues();
    EXPECT_NEAR(Distribution500::eigen_values[0], eigen_values(1), tolerance);
    EXPECT_NEAR(Distribution500::eigen_values[1], eigen_values(0), tolerance);

    distribution.reset();
    for(std::size_t i = 0 ; i < Distribution5000::size ; ++i) {
        distribution.add({Distribution5000::data[i * 2 + 0],
                          Distribution5000::data[i * 2 + 1]});
    }

    eigen_values = distribution.getEigenValues();
    EXPECT_NEAR(Distribution5000::eigen_values[0], eigen_values(0), tolerance);
    EXPECT_NEAR(Distribution5000::eigen_values[1], eigen_values(1), tolerance);
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
    for(std::size_t i = 0 ; i < Distribution200::size ; ++i) {
        distribution.add({Distribution200::data[i * 2 + 0],
                          Distribution200::data[i * 2 + 1]});
    }

    eigen_vectors = distribution.getEigenVectors();

    Eigen::Vector2d exp_a = {Distribution200::eigen_vectors[0],
                             Distribution200::eigen_vectors[2]};
    Eigen::Vector2d exp_b = {Distribution200::eigen_vectors[1],
                             Distribution200::eigen_vectors[3]};

    Eigen::Vector2d rec_a = eigen_vectors.col(0);
    Eigen::Vector2d rec_b = eigen_vectors.col(1);
    /// direction and storage order of vectors has not to euqal
    bool condition = (equals(exp_a, rec_a, tolerance) && equals(exp_b, rec_b, tolerance)) ||
                     (equals(exp_a, rec_b, tolerance) && equals(exp_b, rec_a, tolerance));

    EXPECT_TRUE(condition);

    /// 500
    distribution.reset();
    for(std::size_t i = 0 ; i < Distribution500::size ; ++i) {
        distribution.add({Distribution500::data[i * 2 + 0],
                          Distribution500::data[i * 2 + 1]});
    }

    eigen_vectors = distribution.getEigenVectors();

    exp_a = {Distribution500::eigen_vectors[0],
             Distribution500::eigen_vectors[2]};
    exp_b = {Distribution500::eigen_vectors[1],
             Distribution500::eigen_vectors[3]};

    rec_a = eigen_vectors.col(0);
    rec_b = eigen_vectors.col(1);
    /// direction and storage order of vectors has not to euqal
    condition = (equals(exp_a, rec_a, tolerance) && equals(exp_b, rec_b, tolerance)) ||
                (equals(exp_a, rec_b, tolerance) && equals(exp_b, rec_a, tolerance));

    EXPECT_TRUE(condition);



    /// 5000
    distribution.reset();
    for(std::size_t i = 0 ; i < Distribution5000::size ; ++i) {
        distribution.add({Distribution5000::data[i * 2 + 0],
                          Distribution5000::data[i * 2 + 1]});
    }

    eigen_vectors = distribution.getEigenVectors();

    exp_a = {Distribution5000::eigen_vectors[0],
             Distribution5000::eigen_vectors[2]};
    exp_b = {Distribution5000::eigen_vectors[1],
             Distribution5000::eigen_vectors[3]};

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
    for(std::size_t i = 0 ; i < Distribution200::size ; ++i) {
        distribution_a.add({Distribution200::data[i * 2 + 0],
                            Distribution200::data[i * 2 + 1]});
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
    for(std::size_t i = 0 ; i < Distribution200::size ; ++i) {
        distribution_a.add({Distribution200::data[i * 2 + 0],
                            Distribution200::data[i * 2 + 1]});
    }

    distribution_b  = distribution_a;
    distribution_b += distribution_a;

    for(std::size_t i = 0 ; i < Distribution200::size ; ++i) {
        distribution_a.add({Distribution200::data[i * 2 + 0],
                            Distribution200::data[i * 2 + 1]});
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
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}


