#include <gtest/gtest.h>

#include <muse_amcl/math/covariance.hpp>

using Matrix6d = Eigen::Matrix<double, 6, 6>;
using Matrix3d = Eigen::Matrix<double, 3, 3>;
using Covariance = muse_mcl::math::Covariance;

TEST(TestMuseAMCL, testDefaultConstructor)
{
    Covariance cov;
    EXPECT_EQ(cov.getData() , Matrix6d::Zero());
    EXPECT_EQ(cov.getEigen3D() , Matrix3d::Zero());
}

TEST(TestMuseAMCL, testConstructor3d)
{
    auto makeIt6D = [](const Matrix3d &mat)
    {
        Matrix6d mat_6d = Matrix6d::Zero();
        /// set up x and y related values
        mat_6d.block<2,2>(0,0) = mat.block<2,2>(0,0);
        /// set up the yaw angle
        mat_6d(5,0) = mat(2,0);
        mat_6d(5,1) = mat(2,1);
        mat_6d(0,5) = mat(0,2);
        mat_6d(1,5) = mat(1,2);
        mat_6d(5,5) = mat(2,2);
        return mat_6d;
    };

    const Matrix3d data3D = Matrix3d::Random();
    const Matrix6d data6D = makeIt6D(data3D);

    Covariance cov(data3D);
    EXPECT_EQ(cov.getData() , data6D);
    EXPECT_EQ(cov.getEigen3D() , data3D);
}

TEST(TestMuseAMCL, testConstructor6d)
{
    auto makeIt3D = [](const Matrix6d &mat) {
        Matrix3d mat_3d = Matrix3d::Zero();
        /// set up x and y related values
        mat_3d.block<2,2>(0,0) = mat.block<2,2>(0,0);
        /// set up the yaw angle
        mat_3d(2,0) = mat(5,0);
        mat_3d(2,1) = mat(5,1);
        mat_3d(0,2) = mat(0,5);
        mat_3d(1,2) = mat(1,5);
        mat_3d(2,2) = mat(5,5);
        return mat_3d;
    };

    const Matrix6d data6D = Matrix6d::Random();
    const Matrix3d data3D = makeIt3D(data6D);

    Covariance cov(data6D);
    EXPECT_EQ(cov.getData() , data6D);
    EXPECT_EQ(cov.getEigen3D() , data3D);
}

TEST(TestMuseAMCL, testConstructorStd3d)
{
    auto makeIt6D = [](const Matrix3d &mat)
    {
        Matrix6d mat_6d = Matrix6d::Zero();
        /// set up x and y related values
        mat_6d.block<2,2>(0,0) = mat.block<2,2>(0,0);
        /// set up the yaw angle
        mat_6d(5,0) = mat(2,0);
        mat_6d(5,1) = mat(2,1);
        mat_6d(0,5) = mat(0,2);
        mat_6d(1,5) = mat(1,2);
        mat_6d(5,5) = mat(2,2);
        return mat_6d;
    };

    const Matrix3d data3D = Matrix3d::Random();
    const Matrix6d data6D = makeIt6D(data3D);
    std::vector<double> data;
    for(std::size_t i = 0 ; i < 3 ; ++i) {
        for(std::size_t j = 0 ; j < 3 ; ++j) {
            data.emplace_back(data3D(i,j));
        }
    }

    Covariance cov(data);
    EXPECT_EQ(cov.getData() , data6D);
    EXPECT_EQ(cov.getEigen3D() , data3D);
}

TEST(TestMuseAMCL, testConstructorStd6d)
{
    auto makeIt3D = [](const Matrix6d &mat) {
        Matrix3d mat_3d = Matrix3d::Zero();
        /// set up x and y related values
        mat_3d.block<2,2>(0,0) = mat.block<2,2>(0,0);
        /// set up the yaw angle
        mat_3d(2,0) = mat(5,0);
        mat_3d(2,1) = mat(5,1);
        mat_3d(0,2) = mat(0,5);
        mat_3d(1,2) = mat(1,5);
        mat_3d(2,2) = mat(5,5);
        return mat_3d;
    };

    const Matrix6d data6D = Matrix6d::Random();
    const Matrix3d data3D = makeIt3D(data6D);
    std::vector<double> data;
    for(std::size_t i = 0 ; i < 6 ; ++i) {
        for(std::size_t j = 0 ; j < 6 ; ++j) {
            data.emplace_back(data6D(i,j));
        }
    }

    Covariance cov(data);
    EXPECT_EQ(cov.getData(), data6D);
    EXPECT_EQ(cov.getEigen3D(), data3D);
}

int main(int argc, char *argv[])
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
