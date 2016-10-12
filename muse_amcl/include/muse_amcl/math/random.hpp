#pragma once

#include <cmath>
#include <random>
#include <memory>
#include <array>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Eigen>

namespace muse {
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
        random_device(),
        random_engine(random_device())
    {
    }

    RandomGenerator(const RandomGenerator &other) = delete;


    std::random_device         random_device;
    std::default_random_engine random_engine;

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

    Uniform(const Vector &_min,
            const Vector &_max)
    {
        set(_min, _max);
    }

    inline void set(const Vector &_min,
                    const Vector &_max)
    {
        for(std::size_t i = 0 ;  i < Dim ; ++i) {
            distributions[i] = Distribution(_min[i], _max[i]);
        }
    }

    inline Vector get()
    {
        Vector sample;
        for(std::size_t i = 0 ; i < Dim ; ++i) {
            sample[i] = distributions[i](random_engine);
        }
        return sample;
    }

    inline void get(Vector &_sample)
    {
        for(std::size_t i = 0 ; i < Dim ; ++i) {
            _sample[i] = distributions[i](random_engine);
        }
    }

private:
    std::array<Distribution, Dim> distributions;
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

    Uniform(const double _min,
            const double _max)
    {
        set(_min, _max);
    }

    inline void set(const double _min,
                    const double _max)
    {
        distribution = Distribution(_min, _max);
    }

    inline double get()
    {
        return distribution(random_engine);
    }

    inline void get(double &_sample)
    {
        _sample = distribution(random_engine);
    }

private:
    Distribution distribution;

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

    Normal(const Vector &_mean,
             const Matrix &_covariance)
    {
        set(_mean, _covariance);
    }

    inline void set(const Vector &_mean,
                    const Matrix &_covariance)
    {
        mean = _mean;
        covariance = _covariance;

        EigenSolver eigen(covariance);
        rotation = eigen.eigenvectors().real();           /// rotation into the "world_frame"
        scale = eigen.eigenvalues().real().cwiseSqrt();   /// scale along the main axis of distribution
    }

    inline Vector get()
    {
        Vector sample;
        for(std::size_t i = 0 ; i < Dim ; ++i)
            sample(i) = distribution(random_engine) * scale(i);
        return rotation * sample + mean;
    }

    inline void get(Vector &_sample)
    {
        for(std::size_t i = 0 ; i < Dim ; ++i)
            _sample(i) = distribution(random_engine) * scale(i);
        _sample = rotation * _sample + mean;
    }

private:
    Distribution distribution;
    Vector mean;
    Matrix covariance;
    Matrix rotation;
    Vector scale;
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

    Normal(const double _mean,
             const double _sigma)
    {
        set(_mean, _sigma);
    }

    inline void set(const double _mean,
                    const double _sigma)
    {
        distribution = Distribution(_mean, _sigma);
    }

    inline double get()
    {
        return distribution(random_engine);
    }

    inline void get(double &_sample)
    {
        _sample = distribution(random_engine);
    }

private:
    Distribution distribution;
};
}
}
}
