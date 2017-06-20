#include <muse_mcl/utils/buffered_vector.hpp>

#include <gtest/gtest.h>

std::buffered_vector<double> test_vector;

TEST(TestMuseMCL, testInitialization)
{
    EXPECT_EQ(0, test_vector.size());
    EXPECT_EQ(0, test_vector.capacity());

    test_vector = std::buffered_vector<double>(10);

    EXPECT_EQ(10, test_vector.size());
    EXPECT_EQ(10, test_vector.capacity());

    test_vector = std::buffered_vector<double>();
    EXPECT_EQ(0, test_vector.size());
    EXPECT_EQ(0, test_vector.capacity());

    test_vector = std::buffered_vector<double>(0, 10);

    EXPECT_EQ(0, test_vector.size());
    EXPECT_EQ(10, test_vector.capacity());
}

TEST(TestMuseMCL, testResize)
{
    test_vector.resize(99);
    EXPECT_EQ(99, test_vector.size());
    EXPECT_EQ(99, test_vector.capacity());

    test_vector.resize(0,111);
    EXPECT_EQ(0, test_vector.size());
    EXPECT_EQ(111, test_vector.capacity());
}

TEST(TestMuseMCL, testPushback)
{
    for(std::size_t i = 0 ; i < 55 ; ++i) {
        test_vector.push_back(1.0 / i);
        EXPECT_EQ(i+1, test_vector.size());
        EXPECT_EQ(111, test_vector.capacity());
        EXPECT_EQ(1.0 / i, test_vector.back());
    }
}

TEST(TestMuseMCL, testEmplaceBack)
{
    for(std::size_t i = 1 ; i <= 56 ; ++i) {
        test_vector.push_back(1.0 / i);
        EXPECT_EQ(55+i, test_vector.size());
        EXPECT_EQ(111, test_vector.capacity());
        EXPECT_EQ(1.0 / i, test_vector.back());
    }
}

TEST(TestMuseMCL, testVectorIterator)
{
    std::vector<double> t;
    EXPECT_EQ(t.end(), t.begin());
    EXPECT_LT(t.end(),  t.begin() + 1);
    EXPECT_GT(t.rend(), t.rbegin() + 1);
}


int main(int argc, char *argv[])
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
