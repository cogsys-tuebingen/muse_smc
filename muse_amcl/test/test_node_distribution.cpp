#include <muse_amcl/math/distribution.hpp>

#include <gtest/gtest.h>
#include <ros/ros.h>

#include <fstream>
#include <yaml-cpp/yaml.h>

struct TestDistribution {

    void write(const std::string &path)
    {
        std::ofstream out(path);
        YAML::Emitter yaml;
        yaml << YAML::BeginMap;
        write<2,2>("covariance", covariance, yaml);
        write<2,1>("mean", mean, yaml);
        write<2,1>("eigen_values", eigen_values, yaml);
        write<2,2>("eigen_vectors", eigen_vectors, yaml);
        write<2,1>("data", data, yaml);
        yaml << YAML::EndMap;

        out << yaml.c_str();
        out.close();
    }

    template<std::size_t rows, std::size_t cols>
    void write(const std::string &name,
               const std::vector<Eigen::Matrix<double, rows, cols>> &mats,
               YAML::Emitter &yaml)
    {
        yaml << YAML::Key << name << YAML::BeginSeq;
        for(auto &mat : mats) {
            yaml << YAML::BeginMap;
            yaml << YAML::Key << "rows" << YAML::Value << rows;
            yaml << YAML::Key << "cols" << YAML::Value << cols;
            yaml << YAML::Key << "data" << YAML::Value << YAML::BeginSeq;
            for(std::size_t i = 0 ; i < rows; ++i) {
                for(std::size_t j = 0 ; j < cols ; ++j) {
                    yaml << mat(i,j);
                }
            }
            yaml << YAML::EndSeq;
            yaml << YAML::EndMap;
        }
        yaml << YAML::EndSeq;
    }

    template<std::size_t rows, std::size_t cols>
    void write(const std::string &name,
               const Eigen::Matrix<double, rows, cols> &mat,
               YAML::Emitter &yaml)
    {
        yaml << YAML::Key << name << YAML::BeginMap;
        yaml << YAML::Key << "rows" << YAML::Value << rows;
        yaml << YAML::Key << "cols" << YAML::Value << cols;
        yaml << YAML::Key << "data" << YAML::Value << YAML::BeginSeq;
        for(std::size_t i = 0 ; i < rows; ++i) {
            for(std::size_t j = 0 ; j < cols ; ++j) {
                yaml << mat(i,j);
            }
        }
        yaml << YAML::EndSeq;
        yaml << YAML::EndMap;
    }

    void read(const std::string &path)
    {
        YAML::Node yaml = YAML::LoadFile(path);
        assert(yaml.Type() == YAML::NodeType::Map);
        read<2,2>(yaml["covariance"], covariance);
        read<2,1>(yaml["mean"], mean);
        read<2,1>(yaml["eigen_values"], eigen_values);
        read<2,2>(yaml["eigen_vectors"], eigen_vectors);
        read<2,1>(yaml["data"], data);
    }

    template<std::size_t rows, std::size_t cols>
    void read(const YAML::Node  &yaml,
              Eigen::Matrix<double, rows, cols> &mat)
    {
        YAML::const_iterator it = yaml["data"].begin();
        for(std::size_t i = 0 ; i < rows; ++i) {
            for(std::size_t j = 0 ; j < cols ; ++j) {
                mat(i,j) = it->as<double>();
                ++it;
            }
        }
    }

    template<std::size_t rows, std::size_t cols>
    void read(const YAML::Node &yaml,
              std::vector<Eigen::Matrix<double, rows, cols>> &mats)
    {
        for(YAML::const_iterator it = yaml.begin() ; it != yaml.end() ; ++it) {
            Eigen::Matrix<double, rows, cols> mat;
            read<rows, cols>(*it, mat);
            mats.emplace_back(mat);
        }
    }


    Eigen::Matrix2d covariance;
    Eigen::Vector2d mean;
    Eigen::Vector2d eigen_values;
    Eigen::Matrix2d eigen_vectors;
    std::vector<Eigen::Vector2d> data;
};

TestDistribution test_distribution_200;
TestDistribution test_distribution_500;
TestDistribution test_distribution_5000;


namespace mms = muse_amcl::math::statistic;

TEST(TestMuseAMCL, testDistributionInsertion)
{
    mms::Distribution<2> distribution;
    EXPECT_EQ(0, distribution.getN());
    for(std::size_t i = 0 ; i < test_distribution_200.data.size() ; ++i) {
        EXPECT_EQ(i, distribution.getN());
        distribution.add(test_distribution_200.data[i]);
    }
    EXPECT_EQ(test_distribution_200.data.size(), distribution.getN());

    distribution.reset();
    EXPECT_EQ(0, distribution.getN());
    for(std::size_t i = 0 ; i < test_distribution_500.data.size() ; ++i) {
        EXPECT_EQ(i, distribution.getN());
        distribution.add(test_distribution_500.data[i]);
    }
    EXPECT_EQ(test_distribution_500.data.size(), distribution.getN());

    distribution.reset();
    EXPECT_EQ(0, distribution.getN());
    for(std::size_t i = 0 ; i < test_distribution_5000.data.size() ; ++i) {
        EXPECT_EQ(i, distribution.getN());
        distribution.add(test_distribution_5000.data[i]);
    }
    EXPECT_EQ(test_distribution_5000.data.size(), distribution.getN());

}

TEST(TestMuseAMCL, testDistributionMean)
{
    const double tolerance = 1e-3;

    mms::Distribution<2> distribution;
    Eigen::Vector2d mean;
    for(std::size_t i = 0 ; i < test_distribution_200.data.size() ; ++i) {
        distribution.add(test_distribution_200.data[i]);
    }

    mean = distribution.getMean();

    EXPECT_NEAR(test_distribution_200.mean(0), mean(0), tolerance);
    EXPECT_NEAR(test_distribution_200.mean(1), mean(1), tolerance);

    distribution.reset();
    for(std::size_t i = 0 ; i < test_distribution_500.data.size() ; ++i) {
        distribution.add(test_distribution_500.data[i]);
    }

    mean = distribution.getMean();
    EXPECT_NEAR(test_distribution_500.mean(0), mean(0), tolerance);
    EXPECT_NEAR(test_distribution_500.mean(1), mean(1), tolerance);

    distribution.reset();
    for(std::size_t i = 0 ; i < test_distribution_5000.data.size() ; ++i) {
        distribution.add(test_distribution_5000.data[i]);
    }

    mean = distribution.getMean();
    EXPECT_NEAR(test_distribution_5000.mean(0), mean(0), tolerance);
    EXPECT_NEAR(test_distribution_5000.mean(1), mean(1), tolerance);
}

TEST(TestMuseAMCL, testDistributionCovariance)
{
    const double tolerance = 1e-3;

    mms::Distribution<2> distribution;
    Eigen::Matrix2d cov;
    for(std::size_t i = 0 ; i < test_distribution_200.data.size() ; ++i) {
        distribution.add(test_distribution_200.data[i]);
    }

    cov = distribution.getCovariance();

    EXPECT_NEAR(test_distribution_200.covariance(0,0), cov(0,0), tolerance);
    EXPECT_NEAR(test_distribution_200.covariance(0,1), cov(0,1), tolerance);
    EXPECT_NEAR(test_distribution_200.covariance(1,0), cov(1,0), tolerance);
    EXPECT_NEAR(test_distribution_200.covariance(1,1), cov(1,1), tolerance);

    distribution.reset();
    for(std::size_t i = 0 ; i < test_distribution_500.data.size() ; ++i) {
        distribution.add(test_distribution_500.data[i]);
    }

    cov = distribution.getCovariance();
    EXPECT_NEAR(test_distribution_500.covariance(0,0), cov(0,0), tolerance);
    EXPECT_NEAR(test_distribution_500.covariance(0,1), cov(0,1), tolerance);
    EXPECT_NEAR(test_distribution_500.covariance(1,0), cov(1,0), tolerance);
    EXPECT_NEAR(test_distribution_500.covariance(1,1), cov(1,1), tolerance);

    distribution.reset();
    for(std::size_t i = 0 ; i < test_distribution_5000.data.size() ; ++i) {
        distribution.add(test_distribution_5000.data[i]);
    }

    cov = distribution.getCovariance();
    EXPECT_NEAR(test_distribution_5000.covariance(0,0), cov(0,0), tolerance);
    EXPECT_NEAR(test_distribution_5000.covariance(0,1), cov(0,1), tolerance);
    EXPECT_NEAR(test_distribution_5000.covariance(1,0), cov(1,0), tolerance);
    EXPECT_NEAR(test_distribution_5000.covariance(1,1), cov(1,1), tolerance);
}

TEST(TestMuseAMCL, testDistributionEigenValues)
{
    const double tolerance = 1e-3;

    mms::Distribution<2> distribution;
    Eigen::Vector2d eigen_values;
    for(std::size_t i = 0 ; i < test_distribution_200.data.size() ; ++i) {
        distribution.add(test_distribution_200.data[i]);
    }

    eigen_values = distribution.getEigenValues();

    EXPECT_NEAR(test_distribution_200.eigen_values(0), eigen_values(0), tolerance);
    EXPECT_NEAR(test_distribution_200.eigen_values(1), eigen_values(1), tolerance);

    distribution.reset();
    for(std::size_t i = 0 ; i < test_distribution_500.data.size() ; ++i) {
        distribution.add(test_distribution_500.data[i]);
    }

    eigen_values = distribution.getEigenValues();
    EXPECT_NEAR(test_distribution_500.eigen_values(0), eigen_values(1), tolerance);
    EXPECT_NEAR(test_distribution_500.eigen_values(1), eigen_values(0), tolerance);

    distribution.reset();
    for(std::size_t i = 0 ; i < test_distribution_5000.data.size() ; ++i) {
        distribution.add(test_distribution_5000.data[i]);
    }

    eigen_values = distribution.getEigenValues();
    EXPECT_NEAR(test_distribution_5000.eigen_values(0), eigen_values(0), tolerance);
    EXPECT_NEAR(test_distribution_5000.eigen_values(1), eigen_values(1), tolerance);
}

TEST(TestMuseAMCL, testDistributionEigenVectors)
{
    auto equals = [] (const Eigen::Vector2d &a,
            const Eigen::Vector2d &b,
            const double eps) {

        Eigen::Matrix2d invert_direction = Eigen::Matrix2d::Identity() * -1;
        Eigen::Vector2d diff_a = a - b;
        Eigen::Vector2d diff_b = a - invert_direction * b;

        return (fabs(diff_a(0)) <= eps && fabs(diff_a(1)) <= eps) ||
               (fabs(diff_b(0)) <= eps && fabs(diff_b(1)) <= eps);
    };

    const double tolerance = 1e-3;

    mms::Distribution<2> distribution;
    Eigen::Matrix2d eigen_vectors;
    for(std::size_t i = 0 ; i < test_distribution_200.data.size() ; ++i) {
        distribution.add(test_distribution_200.data[i]);
    }

    eigen_vectors = distribution.getEigenVectors();

    Eigen::Vector2d exp_a = test_distribution_200.eigen_vectors.col(0);
    Eigen::Vector2d exp_b = test_distribution_200.eigen_vectors.col(1);

    Eigen::Vector2d rec_a = eigen_vectors.col(0);
    Eigen::Vector2d rec_b = eigen_vectors.col(1);
    /// direction and storage order of vectors has not to euqal
    bool condition = (equals(exp_a, rec_a, tolerance) && equals(exp_b, rec_b, tolerance)) ||
                     (equals(exp_a, rec_b, tolerance) && equals(exp_b, rec_a, tolerance));

    EXPECT_TRUE(condition);

    /// 500
    distribution.reset();
    for(std::size_t i = 0 ; i < test_distribution_500.data.size() ; ++i) {
        distribution.add(test_distribution_500.data[i]);
    }

    eigen_vectors = distribution.getEigenVectors();

    exp_a = test_distribution_500.eigen_vectors.col(0);
    exp_b = test_distribution_500.eigen_vectors.col(1);

    rec_a = eigen_vectors.col(0);
    rec_b = eigen_vectors.col(1);
    /// direction and storage order of vectors has not to euqal
    condition = (equals(exp_a, rec_a, tolerance) && equals(exp_b, rec_b, tolerance)) ||
                (equals(exp_a, rec_b, tolerance) && equals(exp_b, rec_a, tolerance));

    EXPECT_TRUE(condition);



    /// 5000
    distribution.reset();
    for(std::size_t i = 0 ; i < test_distribution_5000.data.size() ; ++i) {
        distribution.add(test_distribution_5000.data[i]);
    }

    eigen_vectors = distribution.getEigenVectors();

    exp_a = test_distribution_5000.eigen_vectors.col(0);
    exp_b = test_distribution_5000.eigen_vectors.col(1);

    rec_a = eigen_vectors.col(0);
    rec_b = eigen_vectors.col(1);
    /// direction and storage order of vectors has not to euqal
    condition = (equals(exp_a, rec_a, tolerance) && equals(exp_b, rec_b, tolerance)) ||
                (equals(exp_a, rec_b, tolerance) && equals(exp_b, rec_a, tolerance));

    EXPECT_TRUE(condition);

}

TEST(TestMuseAMCL, testDistributionCopy)
{
    mms::Distribution<2> distribution_a;
    mms::Distribution<2> distribution_b;
    for(std::size_t i = 0 ; i < test_distribution_200.data.size() ; ++i) {
        distribution_a.add(test_distribution_200.data[i]);
    }

    distribution_b = distribution_a;

    EXPECT_TRUE(distribution_a.getMean() == distribution_b.getMean());
    EXPECT_TRUE(distribution_a.getN()    == distribution_b.getN());

    EXPECT_TRUE(distribution_a.getCovariance() == distribution_b.getCovariance());
    EXPECT_TRUE(distribution_a.getInformationMatrix()    == distribution_b.getInformationMatrix());

    EXPECT_TRUE(distribution_a.getEigenValues() == distribution_b.getEigenValues());
    EXPECT_TRUE(distribution_a.getEigenVectors()   == distribution_b.getEigenVectors());
}


TEST(TestMuseAMCL, testDistributionAddition)
{
    const double tolerance = 1e-6;

    mms::Distribution<2> distribution_a;
    mms::Distribution<2> distribution_b;
    for(std::size_t i = 0 ; i < test_distribution_200.data.size() ; ++i) {
        distribution_a.add(test_distribution_200.data[i]);
    }

    distribution_b  = distribution_a;
    distribution_b += distribution_a;

    for(std::size_t i = 0 ; i < test_distribution_200.data.size() ; ++i) {
        distribution_a.add(test_distribution_200.data[i]);
    }

    std::cout << (distribution_a.getMean() - distribution_b.getMean()).norm() << std::endl;

    EXPECT_NEAR((distribution_a.getMean() - distribution_b.getMean()).norm(), 0.0, tolerance);
    EXPECT_TRUE(distribution_a.getN() == distribution_b.getN());

    EXPECT_NEAR((distribution_a.getCovariance() - distribution_b.getCovariance()).norm(), 0.0, tolerance);
    EXPECT_NEAR((distribution_a.getInformationMatrix() - distribution_b.getInformationMatrix()).norm(), 0.0, tolerance);

    EXPECT_NEAR((distribution_a.getEigenValues() - distribution_b.getEigenValues()).norm(), 0.0, tolerance);
    EXPECT_NEAR((distribution_a.getEigenVectors() - distribution_b.getEigenVectors()).norm(), 0.0, tolerance);
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "muse_amcl_test_node_distribution");
    ros::NodeHandle nh_private("~");

    std::string test_distribution_200_path;
    std::string test_distribution_500_path;
    std::string test_distribution_5000_path;
    if(!nh_private.getParam("test_distribution_200", test_distribution_200_path)) {
        std::cerr << "[TestNodeDistribution]: Cannot load test_distribution_200!" << std::endl;
        std::cout << test_distribution_200_path << std::endl;
        return -1;
    }
    if(!nh_private.getParam("test_distribution_500", test_distribution_500_path)) {
        std::cerr << "[TestNodeDistribution]: Cannot load test_distribution_500!" << std::endl;
        return -1;
    }
    if(!nh_private.getParam("test_distribution_5000", test_distribution_5000_path)) {
        std::cerr << "[TestNodeDistribution]: Cannot load test_distribution_5000!" << std::endl;
        return -1;
    }

    test_distribution_200.read(test_distribution_200_path);
    test_distribution_500.read(test_distribution_500_path);
    test_distribution_5000.read(test_distribution_5000_path);


    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}


