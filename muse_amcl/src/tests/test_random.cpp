#include <muse_amcl/math/distribution.hpp>
#include <muse_amcl/math/random.hpp>
#include <gtest/gtest.h>

TEST(test_random, test_uniform1D)
{

    const std::size_t N = 10000;
    muse_amcl::math::random::Uniform<1> u1D(-10.0, 10.0, 0);

    double mu = 0.0;
    std::vector<double> vs;
    for(std::size_t i = 0 ; i < N ; ++i) {
        double v = u1D.get();
        EXPECT_LE(-10.0, v);
        EXPECT_GE( 10.0, v);
        mu += v;
        vs.emplace_back(v);
    }

    mu /= N;
    EXPECT_NEAR(mu, 0.0, 1e-2);

    double sigma = 0;
    for(auto v : vs) {
        sigma += (mu - v) * (mu - v);
    }
    sigma /= N;
    sigma = sqrt(sigma);
    /// variance of the continous uniform distribution is V(X) = (b - a)^2 / 12 where
    /// b and a are the interval borders
    /// https://en.wikipedia.org/wiki/Uniform_distribution_(continuous)
    /// Therefore a value around 5,773502692 should be expected for the standard deviation.

    EXPECT_NEAR(sigma, 5.77, 1e-2);
}


TEST(test_random, test_normal1D)
{
    const std::size_t N = 5000;
    muse_amcl::math::random::Normal<1> n1D(0.0, 1.0, 0);
}

TEST(test_random, test_uniform2D)
{
    const std::size_t N = 5000;

}

TEST(test_random, test_normal2D)
{
    const std::size_t N = 5000;


}

int main(int argc, char *argv[])
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
