#include <muse_amcl/math/index.hpp>
#include <gtest/gtest.h>

using namespace muse_amcl::math;

TEST(TestMuseAMCL, testInit)
{
    Index<3> i3;
    EXPECT_EQ(0, i3[0]);
    EXPECT_EQ(0, i3[1]);
    EXPECT_EQ(0, i3[2]);

    Index<4> i4;
    EXPECT_EQ(0, i4[0]);
    EXPECT_EQ(0, i4[1]);
    EXPECT_EQ(0, i4[2]);
    EXPECT_EQ(0, i4[3]);

    Index<4> i4i({1,2,3,4});
    EXPECT_EQ(1, i4i[0]);
    EXPECT_EQ(2, i4i[1]);
    EXPECT_EQ(3, i4i[2]);
    EXPECT_EQ(4, i4i[3]);

}

TEST(TestMuseAMCL, testMax)
{
    Index<3> ia({1,0,1});
    Index<3> ib({0,2,0});

    ia.max(ib);
    EXPECT_EQ(1, ia[0]);
    EXPECT_EQ(2, ia[1]);
    EXPECT_EQ(1, ia[2]);

    ia = {-1, 0, -1};
    ib = {-2, 1, -2};
    Index<3> ic = Index<3>::max(ia, ib);
    EXPECT_EQ(-1, ic[0]);
    EXPECT_EQ( 1, ic[1]);
    EXPECT_EQ(-1, ic[2]);
}

TEST(TestMuseAMCL, testMin)
{
    Index<3> ia({2,0,2});
    Index<3> ib({0,1,0});

    ia.min(ib);
    EXPECT_EQ(0, ia[0]);
    EXPECT_EQ(0, ia[1]);
    EXPECT_EQ(0, ia[2]);

    ia = {-1, 0, -1};
    ib = {-2, 1, -2};
    Index<3> ic = Index<3>::min(ia, ib);
    EXPECT_EQ(-2, ic[0]);
    EXPECT_EQ( 0, ic[1]);
    EXPECT_EQ(-2, ic[2]);
}

TEST(TestMuseAMCL, testMinus)
{
    Index<3> ia({2,0,2});
    Index<3> ib({0,1,0});
    Index<3> ic;

    ic = ia - ib;
    EXPECT_EQ( 2, ic[0]);
    EXPECT_EQ(-1, ic[1]);
    EXPECT_EQ( 2, ic[2]);

    ia = {-1, 0, -1};
    ib = {-2, 1, -2};
    ic = ia - ib;
    EXPECT_EQ( 1, ic[0]);
    EXPECT_EQ(-1, ic[1]);
    EXPECT_EQ( 1, ic[2]);
}

TEST(TestMuseAMCL, testPlus)
{
    Index<3> ia({2,0,2});
    Index<3> ib({0,1,0});
    Index<3> ic;

    ic = ia - ib;
    EXPECT_EQ( 2, ic[0]);
    EXPECT_EQ(-1, ic[1]);
    EXPECT_EQ( 2, ic[2]);

    ia = {-1, 0, -1};
    ib = {-2, 1, -2};
    ic = ia + ib;
    EXPECT_EQ(-3, ic[0]);
    EXPECT_EQ( 1, ic[1]);
    EXPECT_EQ(-3, ic[2]);
}

int main(int argc, char *argv[])
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
