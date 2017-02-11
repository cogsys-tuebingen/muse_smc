#include <muse_amcl/math/distribution.hpp>
#include <muse_amcl/math/random.hpp>
#include <gtest/gtest.h>

TEST(TestMuseAMCL, testRandomUniform1d)
{
    const std::size_t N = 1e6;
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


TEST(TestMuseAMCL, testRandomNormal1d)
{
    const std::size_t N = 1e6;
    muse_amcl::math::random::Normal<1> n1D(0.0, 1.0, 0);

    /// checking the interval values should be in relies on statistical propereties of
    /// gaussian distributions. we can assume that most of our generated values should be
    /// contained within boundard given by 200 sigma ;)

    double mu = 0.0;
    std::vector<double> vs;
    for(std::size_t i = 0 ; i < N ; ++i) {
        double v = n1D.get();
        EXPECT_LE(-100.0, v);
        EXPECT_GE( 100.0, v);
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

    /// sigma should be reconstructable from the given sigma.

    EXPECT_NEAR(sigma, 1.0, 1e-2);
}

TEST(TestMuseAMCL, testRandomUniform2d)
{
    const std::size_t N = 1e7;
    muse_amcl::math::random::Uniform<2> u2D({-10.,-10.},
    {10., 10.},
                                            0);
    muse_amcl::math::statistic::Distribution<2> distribution;
    for(std::size_t i = 0 ; i < N ; ++i) {
        auto v = u2D.get();
        distribution.add(Eigen::Vector2d(v[0],v[1]));
        EXPECT_LE(-10.0, v[0]);
        EXPECT_GE( 10.0, v[0]);
        EXPECT_LE(-10.0, v[1]);
        EXPECT_GE( 10.0, v[1]);
    }
    Eigen::Vector2d mu = distribution.getMean();
    EXPECT_NEAR(mu(0), 0.0, 1e-2);
    EXPECT_NEAR(mu(1), 0.0, 1e-2);

    Eigen::Matrix2d sigma = distribution.getCovariance();
    EXPECT_NEAR(sqrt(sigma(0,0)), 5.77, 1e-2);
    EXPECT_NEAR(sqrt(sigma(1,1)), 5.77, 1e-2);
}

TEST(TestMuseAMCL, testRandomNormal2d)
{
    auto createMatrix = [](std::array<double,4> values)
    {
        Eigen::Matrix2d m;
        m(0,0) = values[0];
        m(0,1) = values[1];
        m(1,0) = values[2];
        m(1,1) = values[3];
        return m;
    };


    const std::size_t N = 1e7;
    {
        muse_amcl::math::statistic::Distribution<2> distribution;

        const Eigen::Vector2d mu = Eigen::Vector2d(0.0, 0.0);
        const Eigen::Matrix2d sigma = createMatrix({1.0, 0.0, 0.0, 1.0});

        muse_amcl::math::random::Normal<2> u2D(mu, sigma, 0);
        for(std::size_t i = 0 ; i < N ; ++i) {
            auto v = u2D.get();
            distribution.add(Eigen::Vector2d(v[0],v[1]));
            /// this test can be easily conducted for a symmetric and uncorrelated
            /// covariance
            EXPECT_LE(-100.0, v[0]);
            EXPECT_GE( 100.0, v[0]);
            EXPECT_LE(-100.0, v[1]);
            EXPECT_GE( 100.0, v[1]);
        }
        Eigen::Vector2d mu_est = distribution.getMean();
        Eigen::Matrix2d sigma_est = distribution.getCovariance();

        EXPECT_NEAR(mu(0), mu_est(0), 1e-2);
        EXPECT_NEAR(mu(1), mu_est(1), 1e-2);
        EXPECT_NEAR(sigma(0,0), sigma_est(0,0), 1e-2);
        EXPECT_NEAR(sigma(0,1), sigma_est(0,1), 1e-2);
        EXPECT_NEAR(sigma(1,0), sigma_est(1,0), 1e-2);
        EXPECT_NEAR(sigma(1,1), sigma_est(1,1), 1e-2);
    }

    {
        muse_amcl::math::statistic::Distribution<2> distribution;

        const Eigen::Vector2d mu = Eigen::Vector2d(0.0, 0.0);
        const Eigen::Matrix2d sigma = createMatrix({1.0, 0.5, 0.5, 0.5});

        /// here we only check for resulting moments of the distribution
        muse_amcl::math::random::Normal<2> u2D(mu, sigma, 0);
        for(std::size_t i = 0 ; i < N ; ++i) {
            auto v = u2D.get();
            distribution.add(Eigen::Vector2d(v[0],v[1]));
        }
        Eigen::Vector2d mu_est = distribution.getMean();
        Eigen::Matrix2d sigma_est = distribution.getCovariance();

        EXPECT_NEAR(mu(0), mu_est(0), 1e-2);
        EXPECT_NEAR(mu(1), mu_est(1), 1e-2);
        EXPECT_NEAR(sigma(0,0), sigma_est(0,0), 1e-2);
        EXPECT_NEAR(sigma(0,1), sigma_est(0,1), 1e-2);
        EXPECT_NEAR(sigma(1,0), sigma_est(1,0), 1e-2);
        EXPECT_NEAR(sigma(1,1), sigma_est(1,1), 1e-2);
    }

    {
        muse_amcl::math::statistic::Distribution<2> distribution;

        const Eigen::Vector2d mu = Eigen::Vector2d(0.0, 0.0);
        const Eigen::Matrix2d sigma = createMatrix({1.0, -0.5, -0.5, 0.5});

        /// here we only check for resulting moments of the distribution
        muse_amcl::math::random::Normal<2> u2D(mu, sigma, 0);
        for(std::size_t i = 0 ; i < N ; ++i) {
            auto v = u2D.get();
            distribution.add(Eigen::Vector2d(v[0],v[1]));
        }
        Eigen::Vector2d mu_est = distribution.getMean();
        Eigen::Matrix2d sigma_est = distribution.getCovariance();

        EXPECT_NEAR(mu(0), mu_est(0), 1e-2);
        EXPECT_NEAR(mu(1), mu_est(1), 1e-2);
        EXPECT_NEAR(sigma(0,0), sigma_est(0,0), 1e-2);
        EXPECT_NEAR(sigma(0,1), sigma_est(0,1), 1e-2);
        EXPECT_NEAR(sigma(1,0), sigma_est(1,0), 1e-2);
        EXPECT_NEAR(sigma(1,1), sigma_est(1,1), 1e-2);
    }


    {
        muse_amcl::math::statistic::Distribution<2> distribution;

        const Eigen::Vector2d mu = Eigen::Vector2d(1.0, 2.0);
        const Eigen::Matrix2d sigma = createMatrix({1.0, -0.5, -0.5, 0.5});

        /// here we only check for resulting moments of the distribution
        muse_amcl::math::random::Normal<2> u2D(mu, sigma, 0);
        for(std::size_t i = 0 ; i < N ; ++i) {
            auto v = u2D.get();
            distribution.add(Eigen::Vector2d(v[0],v[1]));
        }
        Eigen::Vector2d mu_est = distribution.getMean();
        Eigen::Matrix2d sigma_est = distribution.getCovariance();

        EXPECT_NEAR(mu(0), mu_est(0), 1e-2);
        EXPECT_NEAR(mu(1), mu_est(1), 1e-2);
        EXPECT_NEAR(sigma(0,0), sigma_est(0,0), 1e-2);
        EXPECT_NEAR(sigma(0,1), sigma_est(0,1), 1e-2);
        EXPECT_NEAR(sigma(1,0), sigma_est(1,0), 1e-2);
        EXPECT_NEAR(sigma(1,1), sigma_est(1,1), 1e-2);
    }
}



int main(int argc, char *argv[])
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
