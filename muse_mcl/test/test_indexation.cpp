#include <muse_mcl/particle_filter/indexation.hpp>
#include <gtest/gtest.h>

using namespace muse_mcl;

TEST(TestMuseMCL, testResolution)
{
    Indexation ia;
    Indexation::ResolutionType res = ia.getResolution();
    EXPECT_EQ(0.0, res[0]);
    EXPECT_EQ(0.0, res[1]);
    EXPECT_EQ(0.0, res[2]);
    ia.setResolution({23.0, 42.0, 65.0});
    res = ia.getResolution();
    EXPECT_EQ(23.0, res[0]);
    EXPECT_EQ(42.0, res[1]);
    EXPECT_EQ(65.0, res[2]);
}

TEST(TestMuseMCL, testCreation)
{
    Indexation ia({0.5, 0.25, 1.0});
    Indexation::IndexType i = ia.create({1.0, 1.0, 1.0});

    EXPECT_EQ(2.0, i[0]);
    EXPECT_EQ(4.0, i[1]);
    EXPECT_EQ(1.0, i[2]);

    i = ia.create({3.0, 3.0, 3.0});
    EXPECT_EQ(6.0, i[0]);
    EXPECT_EQ(12.0,i[1]);
    EXPECT_EQ(3.0, i[2]);

    ia.setResolution({3.0, 3.0, 3.0});
    i = ia.create({1.0, 2.0, 3.0});
    EXPECT_EQ(0.0, i[0]);
    EXPECT_EQ(0.0, i[1]);
    EXPECT_EQ(1.0, i[2]);
}

TEST(TestMuseMCL, testSize)
{
    Indexation ia({0.5, 0.25, 1.0});
    Indexation::SizeType s = ia.size({1.0, 1.0, 1.0});

    EXPECT_EQ(3, s[0]);
    EXPECT_EQ(5, s[1]);
    EXPECT_EQ(2, s[2]);

    s = ia.size(Indexation::IndexType({-1, -1, -1}),
                Indexation::IndexType({1, 1, 1}));

    EXPECT_EQ(3, s[0]);
    EXPECT_EQ(3, s[1]);
    EXPECT_EQ(3, s[2]);

}

int main(int argc, char *argv[])
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
