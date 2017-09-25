#include <gtest/gtest.h>

#include <muse_mcl_2d/math/transform_2d.hpp>
#include <muse_mcl_2d/math/point_2d.hpp>

#include <muse_smc/math/random.hpp>
#include <muse_smc/math/angle.hpp>

#include <tf/tf.h>

using namespace muse_mcl_2d;
using namespace math;
using rng_t = muse_smc::math::random::Uniform<1>;


TEST(Test_muse_mcl_2d, testTransformInitEye)
{
    Transform2D t_0;
    EXPECT_EQ(t_0.yaw(), 0.0);
    EXPECT_EQ(t_0.tx(),  0.0);
    EXPECT_EQ(t_0.ty(),  0.0);
    EXPECT_EQ(t_0.sine(), 0.0);
    EXPECT_EQ(t_0.cosine(), 1.0);

    Transform2D t_1 = Transform2D(0.0, 0.0);
    EXPECT_EQ(t_1.yaw(), 0.0);
    EXPECT_EQ(t_1.tx(),  0.0);
    EXPECT_EQ(t_1.ty(),  0.0);
    EXPECT_EQ(t_1.sine(), 0.0);
    EXPECT_EQ(t_1.cosine(), 1.0);

    Transform2D t_2 = Transform2D(Vector2D(0.0, 0.0));
    EXPECT_EQ(t_2.yaw(), 0.0);
    EXPECT_EQ(t_2.tx(),  0.0);
    EXPECT_EQ(t_2.ty(),  0.0);
    EXPECT_EQ(t_2.sine(), 0.0);
    EXPECT_EQ(t_2.cosine(), 1.0);

    Transform2D t_3 = Transform2D(0.0, 0.0, 0.0);
    EXPECT_EQ(t_3.yaw(), 0.0);
    EXPECT_EQ(t_3.tx(),  0.0);
    EXPECT_EQ(t_3.ty(),  0.0);
    EXPECT_EQ(t_3.sine(), 0.0);
    EXPECT_EQ(t_3.cosine(), 1.0);

    Transform2D t_4 = Transform2D(Vector2D(0.0, 0.0), 0.0);
    EXPECT_EQ(t_4.yaw(), 0.0);
    EXPECT_EQ(t_4.tx(),  0.0);
    EXPECT_EQ(t_4.ty(),  0.0);
    EXPECT_EQ(t_4.sine(), 0.0);
    EXPECT_EQ(t_4.cosine(), 1.0);
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
    EXPECT_EQ(t_0.sine(), 0.0);
    EXPECT_EQ(t_0.cosine(), 1.0);

    Transform2D t_1 = Transform2D(Vector2D(x, y));
    EXPECT_EQ(t_1.yaw(), 0.0);
    EXPECT_EQ(t_1.tx(),  x);
    EXPECT_EQ(t_1.ty(),  y);
    EXPECT_EQ(t_1.sine(), 0.0);
    EXPECT_EQ(t_1.cosine(), 1.0);

    Transform2D t_2 = Transform2D(x, y, 0.0);
    EXPECT_EQ(t_2.yaw(), 0.0);
    EXPECT_EQ(t_2.tx(),  x);
    EXPECT_EQ(t_2.ty(),  y);
    EXPECT_EQ(t_2.sine(), 0.0);
    EXPECT_EQ(t_2.cosine(), 1.0);

    Transform2D t_3 = Transform2D(Vector2D(x, y), 0.0);
    EXPECT_EQ(t_3.yaw(), 0.0);
    EXPECT_EQ(t_3.tx(),  x);
    EXPECT_EQ(t_3.ty(),  y);
    EXPECT_EQ(t_3.sine(), 0.0);
    EXPECT_EQ(t_3.cosine(), 1.0);
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
    EXPECT_EQ(t_0.sine(), sin);
    EXPECT_EQ(t_0.cosine(), cos);

    Transform2D t_1 = Transform2D(Vector2D(0.0, 0.0), yaw);
    EXPECT_EQ(t_1.yaw(), yaw);
    EXPECT_EQ(t_1.tx(),  0.0);
    EXPECT_EQ(t_1.ty(),  0.0);
    EXPECT_EQ(t_1.sine(), sin);
    EXPECT_EQ(t_1.cosine(), cos);
}

TEST(Test_muse_mcl_2d, testTransformConstructors)
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
    EXPECT_EQ(t_0.sine(), sin);
    EXPECT_EQ(t_0.cosine(), cos);

    Transform2D t_1 = Transform2D(Vector2D(x, y), yaw);
    EXPECT_EQ(t_1.yaw(), yaw);
    EXPECT_EQ(t_1.tx(),  x);
    EXPECT_EQ(t_1.ty(),  y);
    EXPECT_EQ(t_1.sine(), sin);
    EXPECT_EQ(t_1.cosine(), cos);
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
    EXPECT_EQ(t_0.sine(), sin);
    EXPECT_EQ(t_0.cosine(), cos);

    Transform2D t_1;
    t_1.setFrom(x,y,yaw);
    EXPECT_EQ(t_1.yaw(), yaw);
    EXPECT_EQ(t_1.tx(),  x);
    EXPECT_EQ(t_1.ty(),  y);
    EXPECT_EQ(t_1.sine(), sin);
    EXPECT_EQ(t_1.cosine(), cos);
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
    EXPECT_EQ(t_0.sine(), 0.0);
    EXPECT_EQ(t_0.cosine(), 1.0);

    t_0.setYaw(yaw);
    EXPECT_EQ(t_0.yaw(), yaw);
    EXPECT_EQ(t_0.tx(),  x);
    EXPECT_EQ(t_0.ty(),  y);
    EXPECT_EQ(t_0.sine(), sin);
    EXPECT_EQ(t_0.cosine(), cos);
}

TEST(Test_muse_mcl_2d, testTransformTranslation)
{
    rng_t rng(-10.0, 10.0);
    const double x_0 = rng.get();
    const double y_0 = rng.get();
    const double p_x = rng.get();
    const double p_y = rng.get();

    Transform2D t_0(x_0,y_0, 0.0);
    EXPECT_EQ(t_0.yaw(), 0.0);
    EXPECT_EQ(t_0.tx(),  x_0);
    EXPECT_EQ(t_0.ty(),  y_0);
    EXPECT_EQ(t_0.sine(), 0.0);
    EXPECT_EQ(t_0.cosine(), 1.0);

    tf::Transform t_0_tf(tf::createIdentityQuaternion(),
                         tf::Vector3(x_0,y_0,0.0));

    Point2D   p(p_x, p_y);
    tf::Point p_tf(p_x, p_y, 0.0);

    p    = t_0 * p;
    p_tf = t_0_tf * p_tf;
    EXPECT_EQ(p.x(), p_x + x_0);
    EXPECT_EQ(p.y(), p_y + y_0);
    EXPECT_EQ(p.x(), p_tf.x());
    EXPECT_EQ(p.y(), p_tf.y());

    const double x_1 = rng.get();
    const double y_1 = rng.get();

    Transform2D t_1(x_1,y_1, 0.0);
    tf::Transform t_1_tf(tf::createIdentityQuaternion(),
                         tf::Vector3(x_1,y_1,0.0));

    t_1 = t_0 * t_1;
    t_1_tf = t_0_tf * t_1_tf;

    EXPECT_EQ(t_1.tx(), x_1 + x_0);
    EXPECT_EQ(t_1.ty(), y_1 + y_0);
    EXPECT_EQ(t_1.tx(), t_1_tf.getOrigin().x());
    EXPECT_EQ(t_1.ty(), t_1_tf.getOrigin().y());
    EXPECT_EQ(t_1.cosine(), 1.0);
    EXPECT_EQ(t_1.sine(), 0.0);
    EXPECT_EQ(t_1.yaw(), 0.0);
}

TEST(Test_muse_mcl_2d, testTransformRotation)
{
    rng_t rng(-10.0, 10.0);
    const double yaw_0 = muse_smc::math::angle::normalize(rng.get());
    const double sin_0 = std::sin(yaw_0);
    const double cos_0 = std::cos(yaw_0);
    const double p_x = rng.get();
    const double p_y = rng.get();

    Transform2D t_0(0.0,0.0,yaw_0);
    EXPECT_EQ(t_0.yaw(), yaw_0);
    EXPECT_EQ(t_0.tx(),  0.0);
    EXPECT_EQ(t_0.ty(),  0.0);
    EXPECT_EQ(t_0.sine(), sin_0);
    EXPECT_EQ(t_0.cosine(), cos_0);

    tf::Transform t_0_tf = tf::Transform(tf::createQuaternionFromYaw(yaw_0),
                                         tf::Vector3(0.0,0.0,0.0));

    Point2D   p(p_x, p_y);
    tf::Point p_tf(p_x, p_y, 0.0);

    EXPECT_EQ(p.x(), p_tf.x());
    EXPECT_EQ(p.y(), p_tf.y());

    p    = t_0 * p;
    p_tf = t_0_tf * p_tf;

    EXPECT_NEAR(p.x(), p_tf.x(), 1e-5);
    EXPECT_NEAR(p.y(), p_tf.y(), 1e-5);

    const double yaw_1 = muse_smc::math::angle::normalize(rng.get());

    Transform2D t_1(0.0, 0.0, yaw_1);
    tf::Transform t_1_tf(tf::createQuaternionFromYaw(yaw_1),
                         tf::Vector3(0.0,0.0,0.0));

    EXPECT_EQ(t_0.tx(), 0.0);
    EXPECT_EQ(t_0.ty(), 0.0);
    EXPECT_EQ(t_1.tx(), 0.0);
    EXPECT_EQ(t_1.ty(), 0.0);

    t_1 = t_0 * t_1;
    tf::Transform t_1_tf_ = t_0_tf * t_1_tf;

    EXPECT_EQ(t_0.tx(), 0.0);
    EXPECT_EQ(t_0.ty(), 0.0);
    EXPECT_EQ(t_1.tx(), 0.0);
    EXPECT_EQ(t_1.ty(), 0.0);
    EXPECT_NEAR(t_1.yaw(), tf::getYaw(t_1_tf_.getRotation()), 1e-5);

}

TEST(Test_muse_mcl_2d, testTransformFull)
{
    rng_t rng(-10.0, 10.0);
    const double yaw_0 = muse_smc::math::angle::normalize(rng.get());
    const double sin_0 = std::sin(yaw_0);
    const double cos_0 = std::cos(yaw_0);
    const double x_0 = rng.get();
    const double y_0 = rng.get();
    const double p_x = rng.get();
    const double p_y = rng.get();


    Transform2D t_0(x_0,y_0,yaw_0);
    EXPECT_EQ(t_0.yaw(), yaw_0);
    EXPECT_EQ(t_0.tx(),  x_0);
    EXPECT_EQ(t_0.ty(),  y_0);
    EXPECT_EQ(t_0.sine(), sin_0);
    EXPECT_EQ(t_0.cosine(), cos_0);

    tf::Transform t_0_tf(tf::createQuaternionFromYaw(yaw_0),
                         tf::Vector3(x_0, y_0, 0.0));

    Point2D   p(p_x, p_y);
    tf::Point p_tf(p_x, p_y, 0.0);

    EXPECT_EQ(p.x(), p_tf.x());
    EXPECT_EQ(p.y(), p_tf.y());

    p    = t_0 * p;
    p_tf = t_0_tf * p_tf;

    EXPECT_NEAR(p.x(), p_tf.x(), 1e-5);
    EXPECT_NEAR(p.y(), p_tf.y(), 1e-5);

    const double yaw_1 = muse_smc::math::angle::normalize(rng.get());
    const double x_1 = rng.get();
    const double y_1 = rng.get();

    Transform2D t_1(x_1, y_1, yaw_1);
    tf::Transform t_1_tf(tf::createQuaternionFromYaw(yaw_1),
                         tf::Vector3(x_1, y_1, 0.0));

    EXPECT_EQ(t_0.tx(), x_0);
    EXPECT_EQ(t_0.ty(), y_0);
    EXPECT_EQ(t_1.tx(), x_1);
    EXPECT_EQ(t_1.ty(), y_1);

    t_1 = t_0 * t_1;
    tf::Transform t_1_tf_ = t_0_tf * t_1_tf;

    EXPECT_EQ(t_0.tx(),  x_0);
    EXPECT_EQ(t_0.ty(),  y_0);
    EXPECT_NE(t_1.tx(),  x_1);
    EXPECT_NE(t_1.ty(),  y_1);
    EXPECT_NEAR(t_1.tx(),  t_1_tf_.getOrigin().x(), 1e-5);
    EXPECT_NEAR(t_1.ty(),  t_1_tf_.getOrigin().y(), 1e-5);
    EXPECT_NEAR(t_1.yaw(), tf::getYaw(t_1_tf_.getRotation()), 1e-5);

    Transform2D t_2 = t_0 * t_1;
    t_0 *= t_1;
    EXPECT_NE(t_0.tx(),  x_0);
    EXPECT_NE(t_0.ty(),  y_0);
    EXPECT_NEAR(t_0.tx(),  t_2.tx(), 1e-5);
    EXPECT_NEAR(t_0.ty(),  t_2.ty(), 1e-5);
    EXPECT_NEAR(t_0.yaw(), t_2.yaw(), 1e-5);


}

TEST(Test_muse_mcl_2d, testTransformInterpolation)
{
    rng_t rng(-10.0, 10.0);
    const double x_0 = rng.get();
    const double y_0 = rng.get();
    const double yaw_0 = muse_smc::math::angle::normalize(rng.get());

    tf::Transform tf_0(tf::createQuaternionFromYaw(yaw_0),
                       tf::Vector3(x_0, y_0, 0.0));
    tf::Transform tf_0_inverse = tf_0.inverse();

    Transform2D t_0(x_0, y_0, yaw_0);
    Transform2D t_0_inverse = t_0.inverse();

    EXPECT_NEAR(t_0_inverse.tx(), tf_0_inverse.getOrigin().x(), 1e-5);
    EXPECT_NEAR(t_0_inverse.ty(), tf_0_inverse.getOrigin().y(), 1e-5);
    EXPECT_NEAR(t_0_inverse.yaw(), tf::getYaw(tf_0_inverse.getRotation()), 1e-5);
}

TEST(Test_muse_mcl_2d, testTransformInverse)
{
    rng_t rng(-10.0, 10.0);
    const double x_0 = rng.get();
    const double y_0 = rng.get();

    Transform2D   t_0(x_0, y_0, 0.0);
    tf::Transform tf_0(tf::createQuaternionFromYaw(0.0),
                       tf::Vector3(x_0, y_0, 0.0));

    Transform2D t_0_inverse = t_0.inverse();
    tf::Transform tf_0_inverse = tf_0.inverse();

    EXPECT_NEAR(t_0_inverse.tx(), tf_0_inverse.getOrigin().x(), 1e-5);
    EXPECT_NEAR(t_0_inverse.ty(), tf_0_inverse.getOrigin().y(), 1e-5);
    EXPECT_NEAR(t_0_inverse.tx(), -x_0, 1e-5);
    EXPECT_NEAR(t_0_inverse.ty(), -y_0, 1e-5);
    EXPECT_NEAR(t_0_inverse.yaw(), 0.0, 1e-5);
    EXPECT_NEAR(t_0_inverse.sine(), 0.0, 1e-5);
    EXPECT_NEAR(t_0_inverse.cosine(), 1.0, 1e-5);

    const double x_1 = rng.get();
    const double y_1 = rng.get();
    const double yaw_1 = muse_smc::math::angle::normalize(rng.get());
    Transform2D   t_1(x_1, y_1, yaw_1);
    tf::Transform tf_1(tf::createQuaternionFromYaw(yaw_1),
                       tf::Vector3(x_1, y_1, 0.0));

    Transform2D t_1_inverse = t_1.inverse();
    tf::Transform tf_1_inverse = tf_1.inverse();

    EXPECT_NEAR(t_1_inverse.tx(), tf_1_inverse.getOrigin().x(), 1e-5);
    EXPECT_NEAR(t_1_inverse.ty(), tf_1_inverse.getOrigin().y(), 1e-5);
    EXPECT_NEAR(t_1_inverse.yaw(),tf::getYaw(tf_1_inverse.getRotation()), 1e-5);


}

int main(int argc, char *argv[])
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
