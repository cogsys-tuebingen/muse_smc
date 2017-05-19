#include <muse_amcl/particle_filter/particle_set.hpp>
#include <chrono>

#include <gtest/gtest.h>

TEST(TestMuseAMCL, testParticleSetConstructors)
{
    const std::size_t N = 500000;
    const std::size_t MIN = 10;
    const std::size_t MAX = N;
    muse_amcl::ParticleSet particle_set_a("frame", N);
    const muse_amcl::Indexation::IndexType minimum_index(std::numeric_limits<int>::max());
    const muse_amcl::Indexation::IndexType maximum_index(std::numeric_limits<int>::min());

    EXPECT_EQ(particle_set_a.getSize(), N);
    EXPECT_EQ(particle_set_a.getMinimumSize(), N);
    EXPECT_EQ(particle_set_a.getMaximumSize(), N);
    EXPECT_EQ(particle_set_a.getMaximumWeight(),0.0);
    EXPECT_EQ(particle_set_a.getSumOfWeights(), 0.0);
    EXPECT_EQ(particle_set_a.getMinimumIndex() ,minimum_index);
    EXPECT_EQ(particle_set_a.getMaximumIndex(), maximum_index);

    particle_set_a =
            muse_amcl::ParticleSet("frame", N, MIN, MAX);

    EXPECT_EQ(particle_set_a.getSize(), N);
    EXPECT_EQ(particle_set_a.getMinimumSize(), MIN);
    EXPECT_EQ(particle_set_a.getMaximumSize(), MAX);
    EXPECT_EQ(particle_set_a.getMaximumWeight(), 0.0);
    EXPECT_EQ(particle_set_a.getSumOfWeights(), 0.0);
    EXPECT_EQ(particle_set_a.getMinimumIndex() ,minimum_index);
    EXPECT_EQ(particle_set_a.getMaximumIndex(), maximum_index);


    muse_amcl::ParticleSet particle_set_b(particle_set_a);
    EXPECT_EQ(particle_set_b.getSize(), MIN);
    EXPECT_EQ(particle_set_b.getMinimumSize(),   MIN);
    EXPECT_EQ(particle_set_b.getMaximumSize(),   MAX);
    EXPECT_EQ(particle_set_b.getMaximumWeight(), 0.0);
    EXPECT_EQ(particle_set_b.getSumOfWeights(),  0.0);
    EXPECT_EQ(particle_set_b.getMinimumIndex(), minimum_index);
    EXPECT_EQ(particle_set_b.getMaximumIndex(), maximum_index);


    muse_amcl::ParticleSet particle_set_c(particle_set_a, true);
    EXPECT_EQ(particle_set_c.getSize(), N);
    EXPECT_EQ(particle_set_c.getMinimumSize(),   MIN);
    EXPECT_EQ(particle_set_c.getMaximumSize(),   MAX);
    EXPECT_EQ(particle_set_c.getMaximumWeight(), 0.0);
    EXPECT_EQ(particle_set_c.getSumOfWeights(),  0.0);
    EXPECT_EQ(particle_set_c.getMinimumIndex(), minimum_index);
    EXPECT_EQ(particle_set_c.getMaximumIndex(), maximum_index);

}

TEST(TestMuseAMCL, testParticleSetResize)
{
    const std::size_t N = 500000;
    const std::size_t NN = 500;
    const std::size_t MIN = 10;
    const std::size_t MAX = N;
    const muse_amcl::Indexation::IndexType minimum_index(std::numeric_limits<int>::max());
    const muse_amcl::Indexation::IndexType maximum_index(std::numeric_limits<int>::min());

    muse_amcl::ParticleSet particle_set("frame", N);
    particle_set.resize(NN);

    EXPECT_EQ(particle_set.getSize(), NN);
    EXPECT_EQ(particle_set.getMinimumSize(), NN);
    EXPECT_EQ(particle_set.getMaximumSize(), NN);
    EXPECT_EQ(particle_set.getMaximumWeight(), 0.0);
    EXPECT_EQ(particle_set.getMaximumSize(), NN);
    EXPECT_EQ(particle_set.getMaximumWeight(), 0.0);
    EXPECT_EQ(particle_set.getSumOfWeights(),  0.0);
    EXPECT_EQ(particle_set.getMinimumIndex(), minimum_index);
    EXPECT_EQ(particle_set.getMaximumIndex(), maximum_index);

    particle_set =
            muse_amcl::ParticleSet("frame", N);

    particle_set.resize(NN, MIN, MAX);
    EXPECT_EQ(particle_set.getSize(), NN);
    EXPECT_EQ(particle_set.getMinimumSize(), MIN);
    EXPECT_EQ(particle_set.getMaximumSize(), MAX);
    EXPECT_EQ(particle_set.getMaximumWeight(), 0.0);
    EXPECT_EQ(particle_set.getSumOfWeights(),  0.0);
    EXPECT_EQ(particle_set.getMinimumIndex(), minimum_index);
    EXPECT_EQ(particle_set.getMaximumIndex(), maximum_index);

}

TEST(TestMuseAMCL, testParticleSetReserve)
{
    const std::size_t N = 500000;
    const std::size_t NN = 500;
    const std::size_t MIN = 10;
    const std::size_t MAX = N;
    const muse_amcl::Indexation::IndexType minimum_index(std::numeric_limits<int>::max());
    const muse_amcl::Indexation::IndexType maximum_index(std::numeric_limits<int>::min());

    muse_amcl::ParticleSet particle_set("frame", 1);
    particle_set.reserve(NN);

    EXPECT_EQ(particle_set.getSize(), 1);
    EXPECT_EQ(particle_set.getMinimumSize(), NN);
    EXPECT_EQ(particle_set.getMaximumSize(), NN);
    EXPECT_EQ(particle_set.getMaximumWeight(), 0.0);
    EXPECT_EQ(particle_set.getMaximumWeight(), 0.0);
    EXPECT_EQ(particle_set.getSumOfWeights(),  0.0);
    EXPECT_EQ(particle_set.getMinimumIndex(), minimum_index);
    EXPECT_EQ(particle_set.getMaximumIndex(), maximum_index);

    particle_set =
            muse_amcl::ParticleSet("frame", 1);

    particle_set.reserve(MIN, MAX);
    EXPECT_EQ(particle_set.getSize(), 1);
    EXPECT_EQ(particle_set.getMinimumSize(), MIN);
    EXPECT_EQ(particle_set.getMaximumSize(), MAX);
    EXPECT_EQ(particle_set.getMaximumWeight(), 0.0);
    EXPECT_EQ(particle_set.getMaximumWeight(), 0.0);
    EXPECT_EQ(particle_set.getSumOfWeights(),  0.0);
    EXPECT_EQ(particle_set.getMinimumIndex(), minimum_index);
    EXPECT_EQ(particle_set.getMaximumIndex(), maximum_index);

}

TEST(TestMuseAMCL, testParticleSetIterators)
{

}

//TEST(TestMuseAMCL, testParticleSetIterator)
//{
//    const std::size_t N = 50;
//    muse_amcl::ParticleSet particle_set("frame", N);

//    auto &particles = particle_set.getParticles();
//    for(std::size_t i = 0 ; i < N ; ++i) {
//        particles[i].pose_.x() = i;
//        particles[i].weight_ = i;
//    }

//    auto weight_it = particle_set.getWeights().begin();
//    for(std::size_t i = 0 ; i < N ; ++i) {
//        EXPECT_EQ(i, (*weight_it));
//        ++weight_it;
//    }

//    auto pose_it = particle_set.getPoses().begin();
//    for(std::size_t i = 0 ; i < N ; ++i) {
//        EXPECT_EQ(i, ((*pose_it).x()));
//        ++pose_it;
//    }

//}

int main(int argc, char *argv[])
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
