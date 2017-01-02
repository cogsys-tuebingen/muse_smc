#include <muse_amcl/math/angle.hpp>

#include <gtest/gtest.h>

namespace mm = muse_amcl::math;

TEST(test_angles, test_normalization)
{
    for(std::size_t i = -10 ; i < 10 ; ++i) {
        double a = mm::angle::normalize(2 * M_PI * i);
        EXPECT_EQ(0.0, a);
    }

    for(std::size_t i = -10 ; i < 10 ; ++i) {
        double a = mm::angle::normalize(2 * M_PI * i + M_PI);
        EXPECT_EQ(M_PI, a);
    }

    for(std::size_t i = -10 ; i < 10 ; ++i) {
        double a = M_PI / 100.0 * i;
        double na = mm::angle::normalize(a);
        if(fabs(a) < 2 * M_PI)
            EXPECT_EQ(na, a);
        else
            EXPECT_LT(fabs(na), 2 * M_PI);
    }
}

//TEST(test_angles, test_difference)
//{
//
//}

TEST(test_angles, test_to_rad)
{
    // expclicit
    auto to_rad = [](double a){return a / 180.0 * M_PI;};
    for(int i = -360 ; i <= 360 ; ++i) {
        EXPECT_EQ(to_rad(i), mm::angle::toRad(i));
    }
}

TEST(test_angles, test_to_deg)
{
    // expclicit
    auto to_deg = [](double a){return a / M_PI * 180.0;};
    auto to_rad = [](double a){return a / 180.0 * M_PI;};
    for(int i = -360 ; i <= 360 ; ++i) {
        double a = to_rad(i);
        EXPECT_EQ(to_deg(a), mm::angle::fromRad(a));
    }

}


int main(int argc, char *argv[])
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
