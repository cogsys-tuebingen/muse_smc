#include <muse_mcl/math/distribution.hpp>
#include <muse_mcl/pose_samplers/normal.hpp>
#include <muse_mcl/pose_samplers/uniform.hpp>

#include <gtest/gtest.h>

using Metric    = muse_mcl::pose_generation::Metric;
using Radian    = muse_mcl::pose_generation::Radian;
using Normal2D  = muse_mcl::pose_generation::Normal<Metric, Metric, Radian>;
using Uniform2D = muse_mcl::pose_generation::Uniform<Metric, Metric, Radian>;

TEST(TestMuseMCL, testPoseSamplingNormal2D)
{
    const std::size_t N = 1e6;

    Eigen::Vector3d mu (0.0, 0.0, 0.0);
    Eigen::Matrix3d sigma = Eigen::Matrix3d::Identity() * 0.5;
    Normal2D normal(mu, sigma);

    muse_mcl::math::statistic::Distribution<3> distribution;
    for(std::size_t i = 0 ; i < N ; ++i) {
        Eigen::Vector3d pose = normal.get();
        distribution.add(pose);
    }
    Eigen::Vector3d mu_est = distribution.getMean();
    Eigen::Matrix3d sigma_est = distribution.getCovariance();

    EXPECT_NEAR(mu(0), mu_est(0), 1e-2);
    EXPECT_NEAR(mu(1), mu_est(1), 1e-2);
    EXPECT_NEAR(0.0,   mu_est(2), 1e-2);
    EXPECT_NEAR(sigma(0,0), sigma_est(0,0), 1e-2);
    EXPECT_NEAR(sigma(1,1), sigma_est(1,1), 1e-2);
    EXPECT_NEAR(sigma(2,2), sigma_est(2,2), 1e-2);

}

TEST(TestMuseMCL, testPoseSamplingUniform2D)
{
    const std::size_t N = 1e7;
    const double A = 3.04;
    Uniform2D uniform({-10.,-10.,  0},
                       {10., 10.,  2 * M_PI}, 0);

    muse_mcl::math::statistic::Distribution<3> distribution;
    for(std::size_t i = 0 ; i < N ; ++i) {
        Eigen::Vector3d pose = uniform.get();
        EXPECT_LE(-10.0, pose(0));
        EXPECT_LE(-10.0, pose(1));
        EXPECT_LE(-M_PI, pose(2));

        EXPECT_GE(10.0, pose(0));
        EXPECT_GE(10.0, pose(1));
        EXPECT_GT(M_PI, pose(2));

        distribution.add(pose);
    }

    Eigen::Vector3d mu_exp(0.0, 0.0, 0.0);
    Eigen::Vector3d mu_est = distribution.getMean();
    EXPECT_NEAR(mu_exp(0), mu_est(0), 1e-2);
    EXPECT_NEAR(mu_exp(1), mu_est(1), 1e-2);
    EXPECT_NEAR(mu_exp(2), mu_est(2), 1e-2);

    Eigen::Matrix3d sigma = distribution.getCovariance();
    EXPECT_NEAR(sqrt(sigma(0,0)), 5.77, 1e-2);
    EXPECT_NEAR(sqrt(sigma(1,1)), 5.77, 1e-2);
    EXPECT_NEAR(sqrt(sigma(2,2)), 1.81, 1e-2);

}

int main(int argc, char *argv[])
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
