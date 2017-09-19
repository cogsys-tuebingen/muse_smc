#include <gtest/gtest.h>

#include <muse_mcl_2d/math/box_2d.hpp>
#include <muse_mcl_2d/math/transform_2d.hpp>
#include <muse_smc/math/random.hpp>

using namespace muse_mcl_2d;
using rng_t = muse_smc::math::random::Uniform<1>;

TEST(Test_muse_mcl_2d, testBoxConstructors)
{
    rng_t rng(-10.0, 10.0);

    math::Box2D b0;
    EXPECT_EQ(b0.getMin().x(), std::numeric_limits<double>::lowest());
    EXPECT_EQ(b0.getMin().y(), std::numeric_limits<double>::lowest());
    EXPECT_EQ(b0.getMax().x(), std::numeric_limits<double>::max());
    EXPECT_EQ(b0.getMax().y(), std::numeric_limits<double>::max());

    double x0 = rng.get();
    double y0 = rng.get();
    double x1 = rng.get();
    double y1 = rng.get();
    if(x0 > x1)
        std::swap(x0, x1);
    if(y0 > y1)
        std::swap(y0, y1);
    math::Box2D b1(x0,y0,x1,y1);
    EXPECT_EQ(b1.getMin().x(), x0);
    EXPECT_EQ(b1.getMin().y(), y0);
    EXPECT_EQ(b1.getMax().x(), x1);
    EXPECT_EQ(b1.getMax().y(), y1);

    math::Box2D b2({x0,y0},{x1,y1});
    EXPECT_EQ(b1.getMin().x(), x0);
    EXPECT_EQ(b1.getMin().y(), y0);
    EXPECT_EQ(b1.getMax().x(), x1);
    EXPECT_EQ(b1.getMax().y(), y1);
}

TEST(Test_muse_mcl_2d, testBoxIntersects)
{
    rng_t rng_out_sign(0.0, 1.0);
    rng_t rng_out(1.1, 10.0);
    rng_t rng_in(-1.0, 1.0);

    auto generate_outer = [&rng_out, &rng_out_sign]()
    {
        const double sign = rng_out_sign.get() > 0.5 ? 1. : -1.;
        return Point2D(rng_out.get(), rng_out.get()) * sign;
    };
    auto generate_inner = [&rng_in] ()
    {
        return Point2D(rng_in.get(), rng_in.get());
    };


    const math::Box2D bb(-1.0, -1.0, 1.0, 1.0);
    for(std::size_t i = 0 ; i < 1000 ; ++i) {
        EXPECT_TRUE(bb.intersects({generate_inner(), generate_outer()}));
    }

    rng_out.set(10.0, 20.0);
    for(std::size_t i = 0 ; i < 1000 ; ++i) {
        const auto start = Point2D(rng_out.get(), rng_out.get());
        const auto end   = Point2D(rng_out.get(), rng_out.get());
        const double angle_incr = M_PI / 18.0;
        double angle = 0.0;

        for(std::size_t i = 0 ; i < 36 ; ++i, angle+=angle_incr) {
            Transform2D rot(angle);
            EXPECT_FALSE(bb.intersects({rot * start, rot * end}));
        }
    }

}

TEST(Test_muse_mcl_2d, testBoxIntersection)
{

}


int main(int argc, char *argv[])
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
