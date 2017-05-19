#ifndef RANDOM_HPP
#define RANDOM_HPP

#include <cmath>
#include <random>
#include <memory>
#include <array>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Eigen>

namespace muse_mcl {
namespace math {
namespace random {

/**
 * @brief Interface for random generator independent from dimensionality and distribution type.
 */
class RandomGenerator {
public:
    typedef std::shared_ptr<RandomGenerator> Ptr;

protected:
    RandomGenerator() :
        random_device_(),
        random_engine_(random_device_())
    {
    }

    RandomGenerator(const unsigned int seed) :
        random_engine_(seed)
    {
    }

    RandomGenerator(const RandomGenerator &other) = delete;


    std::random_device         random_device_;
    std::default_random_engine random_engine_;

};

/**
 * @brief The multi-dimensional uniform random generator class.
 */
template<std::size_t Dim>
class Uniform : public RandomGenerator
{
public:
    typedef std::shared_ptr<Uniform>               Ptr;
    typedef Eigen::Matrix<double, Dim, 1>          Vector;
    typedef std::uniform_real_distribution<double> Distribution;

    Uniform() = delete;

    Uniform(const Vector &min,
            const Vector &max)
    {
        set(min, max);
    }

    Uniform(const Vector &min,
            const Vector &max,
            const unsigned int seed) :
        RandomGenerator(seed)
    {
        set(min, max);
    }

    inline void set(const Vector &min,
                    const Vector &max)
    {
        for(std::size_t i = 0 ;  i < Dim ; ++i) {
            distributions_[i] = Distribution(min[i], max[i]);
        }
    }

    inline Vector get()
    {
        Vector sample;
        for(std::size_t i = 0 ; i < Dim ; ++i) {
            sample[i] = distributions_[i](random_engine_);
        }
        return sample;
    }

    inline void get(Vector &sample)
    {
        for(std::size_t i = 0 ; i < Dim ; ++i) {
            sample[i] = distributions_[i](random_engine_);
        }
    }

private:
    std::array<Distribution, Dim> distributions_;
};

/**
 * @brief The one-dimensional uniform random generator class.
 */
template<>
class Uniform<1> : public RandomGenerator
{
public:
    typedef std::shared_ptr<Uniform> Ptr;
    typedef std::uniform_real_distribution<double> Distribution;

    Uniform() = delete;

    Uniform(const double min,
            const double max)
    {
        set(min, max);
    }

    Uniform(const double min,
            const double max,
            const unsigned int seed) :
        RandomGenerator(seed)
    {
        set(min, max);
    }


    inline void set(const double min,
                    const double max)
    {
        distribution_ = Distribution(min, max);
    }

    inline double get()
    {
        return distribution_(random_engine_);
    }

    inline void get(double &sample)
    {
        sample = distribution_(random_engine_);
    }

private:
    Distribution distribution_;

};

/**
 * @brief The multi-dimensional normally distributed random generator class.
 */
template<std::size_t Dim>
class Normal : public RandomGenerator
{
public:
    typedef std::shared_ptr<Normal>                Ptr;
    typedef Eigen::Matrix<double, Dim, 1>          Vector;
    typedef Eigen::Matrix<double, Dim, Dim>        Matrix;
    typedef std::normal_distribution<double>       Distribution;
    typedef Eigen::EigenSolver<Matrix>             EigenSolver;

    Normal() = delete;

    Normal(const Vector &mean,
           const Matrix &covariance)
    {
        set(mean, covariance);
    }

    Normal(const Vector &mean,
           const Matrix &covariance,
           const unsigned int seed) :
        RandomGenerator(seed)
    {
        set(mean, covariance);
    }

    inline void set(const Vector &mean,
                    const Matrix &covariance)
    {
        mean_ = mean;
        covariance_ = covariance;

        EigenSolver eigen(covariance_);
        rotation_ = eigen.eigenvectors().real();           /// rotation into the "world_frame"
        scale_ = eigen.eigenvalues().real().cwiseSqrt();   /// scale along the main axis of distribution
    }

    inline Vector get()
    {
        Vector sample;
        for(std::size_t i = 0 ; i < Dim ; ++i)
            sample(i) = distribution_(random_engine_) * scale_(i);
        return rotation_ * sample + mean_;
    }

    inline void get(Vector &sample)
    {
        for(std::size_t i = 0 ; i < Dim ; ++i)
            sample(i) = distribution_(random_engine_) * scale_(i);
        sample = rotation_ * sample + mean_;
    }

private:
    Distribution distribution_;
    Vector mean_;
    Matrix covariance_;
    Matrix rotation_;
    Vector scale_;
};

/**
 * @brief The one-dimensional normally distributed  random generator class.
 */
template<>
class Normal<1> : public RandomGenerator
{
public:
    typedef std::shared_ptr<Normal> Ptr;
    typedef std::normal_distribution<double> Distribution;

    Normal() = delete;

    Normal(const double mean,
           const double _sigma)
    {
        set(mean, _sigma);
    }

    Normal(const double mean,
           const double _sigma,
           const unsigned int seed) :
        RandomGenerator(seed)
    {
        set(mean, _sigma);
    }

    inline void set(const double mean,
                    const double _sigma)
    {
        distribution_ = Distribution(mean, _sigma);
    }

    inline double get()
    {
        return distribution_(random_engine_);
    }

    inline void get(double &sample)
    {
        sample = distribution_(random_engine_);
    }

private:
    Distribution distribution_;
};
}
}
}

#endif /* RANDOM_HPP */
