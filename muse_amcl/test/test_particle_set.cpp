#include <muse_amcl/particle_filter/particle_set.hpp>
#include <chrono>

#include <gtest/gtest.h>

TEST(TestMuseAMCL, testParticleSetInitialization)
{
    const std::size_t N = 500000;
    const std::size_t MIN = 10;
    const std::size_t MAX = N;
    muse_amcl::ParticleSet particles("frame", N);

    EXPECT_EQ(particles.getSize(), N);
    EXPECT_EQ(particles.getMinimumSize(), N);
    EXPECT_EQ(particles.getMaximumSize(), N);
    EXPECT_EQ(particles.getMaximumWeight(), 0.0);

    particles =
            muse_amcl::ParticleSet("frame", N, MIN, MAX);

    EXPECT_EQ(particles.getSize(), N);
    EXPECT_EQ(particles.getMinimumSize(), MIN);
    EXPECT_EQ(particles.getMaximumSize(), MAX);
    EXPECT_EQ(particles.getMaximumWeight(), 0.0);
}

TEST(TestMuseAMCL, testParticleSetResize)
{
    const std::size_t N = 500000;
    const std::size_t NN = 500;
    const std::size_t MIN = 10;
    const std::size_t MAX = N;
    muse_amcl::ParticleSet particles("frame", N);

    particles.resize(NN);

    EXPECT_EQ(particles.getSize(), NN);
    EXPECT_EQ(particles.getMinimumSize(), NN);
    EXPECT_EQ(particles.getMaximumSize(), NN);
    EXPECT_EQ(particles.getMaximumWeight(), 0.0);

    particles =
            muse_amcl::ParticleSet("frame", N);

    particles.resize(NN, MIN, MAX);
    EXPECT_EQ(particles.getSize(), NN);
    EXPECT_EQ(particles.getMinimumSize(), MIN);
    EXPECT_EQ(particles.getMaximumSize(), MAX);
    EXPECT_EQ(particles.getMaximumWeight(), 0.0);
}

TEST(TestMuseAMCL, testParticleSetReserve)
{
    const std::size_t N = 500000;
    const std::size_t NN = 500;
    const std::size_t MIN = 10;
    const std::size_t MAX = N;
    muse_amcl::ParticleSet particles("frame", 0);

    particles.reserve(NN);

    EXPECT_EQ(particles.getSize(), 0);
    EXPECT_EQ(particles.getMinimumSize(), NN);
    EXPECT_EQ(particles.getMaximumSize(), NN);
    EXPECT_EQ(particles.getMaximumWeight(), 0.0);

    particles =
            muse_amcl::ParticleSet("frame", 0);

    particles.reserve(NN, MIN, MAX);
    EXPECT_EQ(particles.getSize(), 0);
    EXPECT_EQ(particles.getMinimumSize(), MIN);
    EXPECT_EQ(particles.getMaximumSize(), MAX);
    EXPECT_EQ(particles.getMaximumWeight(), 0.0);
}

int main(int argc, char *argv[])
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
