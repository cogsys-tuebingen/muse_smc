#include <gtest/gtest.h>

#include <muse_mcl_2d/math/transform_2d.hpp>
#include <muse_smc/math/random.hpp>
#include <muse_smc/math/angle.hpp>
#include <tf/tf.h>

using namespace muse_mcl_2d;
using rng_t = muse_smc::math::random::Uniform<1>;


TEST(Test_muse_mcl_2d, testTransformInitEye)
{
    Transform2D t_0;
    EXPECT_EQ(t_0.yaw(), 0.0);
    EXPECT_EQ(t_0.tx(),  0.0);
    EXPECT_EQ(t_0.ty(),  0.0);
    EXPECT_EQ(t_0.sin(), 0.0);
    EXPECT_EQ(t_0.cos(), 1.0);

    Transform2D t_1 = Transform2D(0.0, 0.0);
    EXPECT_EQ(t_1.yaw(), 0.0);
    EXPECT_EQ(t_1.tx(),  0.0);
    EXPECT_EQ(t_1.ty(),  0.0);
    EXPECT_EQ(t_1.sin(), 0.0);
    EXPECT_EQ(t_1.cos(), 1.0);

    Transform2D t_2 = Transform2D(Vector2D(0.0, 0.0));
    EXPECT_EQ(t_2.yaw(), 0.0);
    EXPECT_EQ(t_2.tx(),  0.0);
    EXPECT_EQ(t_2.ty(),  0.0);
    EXPECT_EQ(t_2.sin(), 0.0);
    EXPECT_EQ(t_2.cos(), 1.0);

    Transform2D t_3 = Transform2D(0.0, 0.0, 0.0);
    EXPECT_EQ(t_3.yaw(), 0.0);
    EXPECT_EQ(t_3.tx(),  0.0);
    EXPECT_EQ(t_3.ty(),  0.0);
    EXPECT_EQ(t_3.sin(), 0.0);
    EXPECT_EQ(t_3.cos(), 1.0);

    Transform2D t_4 = Transform2D(Vector2D(0.0, 0.0), 0.0);
    EXPECT_EQ(t_4.yaw(), 0.0);
    EXPECT_EQ(t_4.tx(),  0.0);
    EXPECT_EQ(t_4.ty(),  0.0);
    EXPECT_EQ(t_4.sin(), 0.0);
    EXPECT_EQ(t_4.cos(), 1.0);
}

TEST(Test_muse_mcl_2d, testTransformInitTranslation)
{
    rng_t rng(-10.0, 10.0);
    const double x = rng.get();
    const double y = rng.get();

    Transform2D t_0 = Transform2D(x, y);
    EXPECT_EQ(t_0.yaw(), 0.0);
    EXPECT_EQ(t_0.tx(),  x);
    EXPECT_EQ(t_0.ty(),  y);
    EXPECT_EQ(t_0.sin(), 0.0);
    EXPECT_EQ(t_0.cos(), 1.0);

    Transform2D t_1 = Transform2D(Vector2D(x, y));
    EXPECT_EQ(t_1.yaw(), 0.0);
    EXPECT_EQ(t_1.tx(),  x);
    EXPECT_EQ(t_1.ty(),  y);
    EXPECT_EQ(t_1.sin(), 0.0);
    EXPECT_EQ(t_1.cos(), 1.0);

    Transform2D t_2 = Transform2D(x, y, 0.0);
    EXPECT_EQ(t_2.yaw(), 0.0);
    EXPECT_EQ(t_2.tx(),  x);
    EXPECT_EQ(t_2.ty(),  y);
    EXPECT_EQ(t_2.sin(), 0.0);
    EXPECT_EQ(t_2.cos(), 1.0);

    Transform2D t_3 = Transform2D(Vector2D(x, y), 0.0);
    EXPECT_EQ(t_3.yaw(), 0.0);
    EXPECT_EQ(t_3.tx(),  x);
    EXPECT_EQ(t_3.ty(),  y);
    EXPECT_EQ(t_3.sin(), 0.0);
    EXPECT_EQ(t_3.cos(), 1.0);
}

TEST(Test_muse_mcl_2d, testTransformInitRotation)
{
    rng_t rng(-10.0, 10.0);
    const double yaw = muse_smc::math::angle::normalize(rng.get());
    const double sin = std::sin(yaw);
    const double cos = std::cos(yaw);

    Transform2D t_0 = Transform2D(0.0, 0.0, yaw);
    EXPECT_EQ(t_0.yaw(), yaw);
    EXPECT_EQ(t_0.tx(),  0.0);
    EXPECT_EQ(t_0.ty(),  0.0);
    EXPECT_EQ(t_0.sin(), sin);
    EXPECT_EQ(t_0.cos(), cos);

    Transform2D t_1 = Transform2D(Vector2D(0.0, 0.0), yaw);
    EXPECT_EQ(t_1.yaw(), yaw);
    EXPECT_EQ(t_1.tx(),  0.0);
    EXPECT_EQ(t_1.ty(),  0.0);
    EXPECT_EQ(t_1.sin(), sin);
    EXPECT_EQ(t_1.cos(), cos);
}

TEST(Test_muse_mcl_2d, testTransformInitFull)
{
    rng_t rng(-10.0, 10.0);
    const double x = rng.get();
    const double y = rng.get();
    const double yaw = muse_smc::math::angle::normalize(rng.get());
    const double sin = std::sin(yaw);
    const double cos = std::cos(yaw);

    Transform2D t_0 = Transform2D(x, y, yaw);
    EXPECT_EQ(t_0.yaw(), yaw);
    EXPECT_EQ(t_0.tx(),  x);
    EXPECT_EQ(t_0.ty(),  y);
    EXPECT_EQ(t_0.sin(), sin);
    EXPECT_EQ(t_0.cos(), cos);

    Transform2D t_1 = Transform2D(Vector2D(x, y), yaw);
    EXPECT_EQ(t_1.yaw(), yaw);
    EXPECT_EQ(t_1.tx(),  x);
    EXPECT_EQ(t_1.ty(),  y);
    EXPECT_EQ(t_1.sin(), sin);
    EXPECT_EQ(t_1.cos(), cos);
}

TEST(Test_muse_mcl_2d, testTransformSetFrom)
{
    rng_t rng(-10.0, 10.0);
    const double x = rng.get();
    const double y = rng.get();
    const double yaw = muse_smc::math::angle::normalize(rng.get());
    const double sin = std::sin(yaw);
    const double cos = std::cos(yaw);

    Eigen::Vector3d e(x,y,yaw);
    Transform2D t_0;
    t_0.setFrom(e);
    EXPECT_EQ(t_0.yaw(), yaw);
    EXPECT_EQ(t_0.tx(),  x);
    EXPECT_EQ(t_0.ty(),  y);
    EXPECT_EQ(t_0.sin(), sin);
    EXPECT_EQ(t_0.cos(), cos);

    Transform2D t_1;
    t_1.setFrom(x,y,yaw);
    EXPECT_EQ(t_1.yaw(), yaw);
    EXPECT_EQ(t_1.tx(),  x);
    EXPECT_EQ(t_1.ty(),  y);
    EXPECT_EQ(t_1.sin(), sin);
    EXPECT_EQ(t_1.cos(), cos);
}


TEST(Test_muse_mcl_2d, testTransformSetYaw)
{
    rng_t rng(-10.0, 10.0);
    const double x = rng.get();
    const double y = rng.get();
    const double yaw = muse_smc::math::angle::normalize(rng.get());
    const double sin = std::sin(yaw);
    const double cos = std::cos(yaw);

    Transform2D t_0(x,y, 0.0);
    EXPECT_EQ(t_0.yaw(), 0.0);
    EXPECT_EQ(t_0.tx(),  x);
    EXPECT_EQ(t_0.ty(),  y);
    EXPECT_EQ(t_0.sin(), 0.0);
    EXPECT_EQ(t_0.cos(), 1.0);

    t_0.setYaw(yaw);
    EXPECT_EQ(t_0.yaw(), yaw);
    EXPECT_EQ(t_0.tx(),  x);
    EXPECT_EQ(t_0.ty(),  y);
    EXPECT_EQ(t_0.sin(), sin);
    EXPECT_EQ(t_0.cos(), cos);
}

TEST(Test_muse_mcl_2d, testTransformTranslation)
{

}

TEST(Test_muse_mcl_2d, testTransformRotation)
{

}

TEST(Test_muse_mcl_2d, testTransformFull)
{

}

TEST(Test_muse_mcl_2d, testTransformInterpolation)
{

}

int main(int argc, char *argv[])
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
