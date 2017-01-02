#include <muse_amcl/math/distribution.hpp>

#include "finite_example_distributions.hpp"

#include <gtest/gtest.h>

#include <fstream>

namespace mms = muse_amcl::math::statistic;

using DataPoint = std::array<double, 2>;
using Distribution200 = example_distributions::Distribution200;
using Distribution500 = example_distributions::Distribution500;
using Distribution5000 = example_distributions::Distribution5000;

TEST(test_distribution, test_insertion)
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

TEST(test_distribution, test_mean)
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

TEST(test_distribution, test_cov)
{

}

TEST(test_distribution, test_eigen_values)
{

}

TEST(test_distribution, test_eigen_vectors)
{

}

int main(int argc, char *argv[])
{

    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}


