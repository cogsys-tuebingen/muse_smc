#include <muse_amcl/particle_filter/particle_set.hpp>
#include <chrono>

#include <gtest/gtest.h>

TEST(test_particle_set, test_initialization)
{
    const std::size_t N = 500000;
    const std::size_t MIN = 10;
    const std::size_t MAX = N;
    muse_amcl::ParticleSet particles(N);

    EXPECT_EQ(particles.size(), N);
    EXPECT_EQ(particles.minimumSize(), N);
    EXPECT_EQ(particles.maximumSize(), N);
    EXPECT_EQ(particles.maximumWeight(), 0.0);

    particles =
            muse_amcl::ParticleSet(N, MIN, MAX);

    EXPECT_EQ(particles.size(), N);
    EXPECT_EQ(particles.minimumSize(), MIN);
    EXPECT_EQ(particles.maximumSize(), MAX);
    EXPECT_EQ(particles.maximumWeight(), 0.0);
}

TEST(test_particle_set, test_resize)
{
    const std::size_t N = 500000;
    const std::size_t NN = 500;
    const std::size_t MIN = 10;
    const std::size_t MAX = N;
    muse_amcl::ParticleSet particles(N);

    particles.resize(NN);

    EXPECT_EQ(particles.size(), NN);
    EXPECT_EQ(particles.minimumSize(), NN);
    EXPECT_EQ(particles.maximumSize(), NN);
    EXPECT_EQ(particles.maximumWeight(), 0.0);

    particles =
            muse_amcl::ParticleSet(N);

    particles.resize(NN, MIN, MAX);
    EXPECT_EQ(particles.size(), NN);
    EXPECT_EQ(particles.minimumSize(), MIN);
    EXPECT_EQ(particles.maximumSize(), MAX);
    EXPECT_EQ(particles.maximumWeight(), 0.0);
}

TEST(test_particle_set, test_reserve)
{
    const std::size_t N = 500000;
    const std::size_t NN = 500;
    const std::size_t MIN = 10;
    const std::size_t MAX = N;
    muse_amcl::ParticleSet particles(0);

    particles.reserve(NN);

    EXPECT_EQ(particles.size(), 0);
    EXPECT_EQ(particles.minimumSize(), NN);
    EXPECT_EQ(particles.maximumSize(), NN);
    EXPECT_EQ(particles.maximumWeight(), 0.0);

    particles =
            muse_amcl::ParticleSet(0);

    particles.reserve(NN, MIN, MAX);
    EXPECT_EQ(particles.size(), 0);
    EXPECT_EQ(particles.minimumSize(), MIN);
    EXPECT_EQ(particles.maximumSize(), MAX);
    EXPECT_EQ(particles.maximumWeight(), 0.0);
}

int main(int argc, char *argv[])
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
