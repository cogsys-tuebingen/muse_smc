#ifndef MATH_HPP
#define MATH_HPP
#include <cmath>
#include <random>
#include <memory>
#include <array>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Eigen>

namespace math {
#define _2_M_PI 2.0 * M_PI

inline double normalize(const double angle )
{
    if(fabs(angle) < _2_M_PI)
        return angle;

    return angle - _2_M_PI * floor( angle / _2_M_PI );
}

inline double angleDiff(double a, double b)
{
    double d1, d2;
    a = normalize(a);
    b = normalize(b);
    d1 = a-b;
    d2 = 2*M_PI - fabs(d1);
    if(d1 > 0)
        d2 *= -1.0;
    if(fabs(d1) < fabs(d2))
        return normalize(d1);
    else
        return normalize(d2);
}


template<typename Scalar, std::size_t Dim>
struct Type {
    template<bool, typename T, std::size_t D>
    struct Distribution {
    };

    template<typename T, std::size_t D>
    struct Distribution<true, T, D> {
        typedef std::uniform_int_distribution<T> Type;
    };
    template<typename T, std::size_t D>
    struct Distribution<false, T, D> {
        typedef std::uniform_real_distribution<T> Type;
    };


    typedef std::random_device                                            RandomDevice;
    typedef std::normal_distribution<>                                    GaussianDistribution;
    typedef typename
    Distribution<std::is_integral<Scalar>::value, Scalar, Dim>::Type      UniformDistribution;
    typedef std::mt19937                                                  RandomEngine;
    typedef Eigen::Matrix<Scalar, Dim, 1>                                 Vector;
    typedef Eigen::Matrix<Scalar, Dim, Dim>                               Matrix;
};


template<typename Scalar, std::size_t Dim>
class Uniform {
public:
    typedef std::shared_ptr<Uniform> Ptr;
    typedef Type<Scalar, Dim>        Types;

    Uniform() :
        engine(rd())
    {
    }

    Uniform(const typename Types::Vector _min,
            const typename Types::Vector _max) :
        engine(rd())
    {
        set(_min, _max);
    }

    void set(const typename Types::Vector _min,
             const typename Types::Vector _max)
    {
        for(std::size_t i = 0 ; i < Dim ; ++i) {
            distributions[i] = typename Types::UniformDistribution(_min(i), _max(i));
        }
    }

    inline typename Types::Vector next()
    {
        typename Types::Vector sample;
        for(std::size_t i = 0 ; i < Dim ; ++i) {
            sample(i) = distributions[i](engine);
        }
        return sample;
    }

private:
    typename  Types::RandomDevice                         rd;
    typename  Types::RandomEngine                         engine;
    std::array<typename Types::UniformDistribution, Dim>  distributions;

};

template<typename Scalar>
class Uniform<Scalar, 1> {
public:
    typedef std::shared_ptr<Uniform> Ptr;
    typedef Type<Scalar, 1>          Types;

    Uniform() :
        engine(rd())
    {
    }

    Uniform(const Scalar _min,
            const Scalar _max) :
        engine(rd())
    {
        set(_min, _max);
    }

    void set(const Scalar _min,
             const Scalar _max)
    {
        distribution = typename Types::UniformDistribution(_min, _max);
    }

    inline Scalar next()
    {
        return distribution(engine);
    }

private:
    typename  Types::RandomDevice        rd;
    typename  Types::RandomEngine        engine;
    typename  Types::UniformDistribution  distribution;

};

template<typename Scalar, std::size_t Dim>
class Gaussian {
public:
    typedef std::shared_ptr<Gaussian> Ptr;
    typedef Type<Scalar, Dim>         Types;

    Gaussian() :
        engine(rd())
    {
    }

    Gaussian(const typename Types::Vector &_mean,
             const typename Types::Matrix &_cov) :
        engine(rd()),
        mean(_mean)
    {
        setCovariance(_cov);
    }

    void setMean(const typename Types::Vector &_mean)
    {
        mean = _mean;
    }

    void setCovariance(const typename Types::Matrix &_cov)
    {
        Eigen::EigenSolver<typename Types::Matrix> solver;
        solver.compute(_cov);
        rotation = solver.eigenvectors().real();
        diagonal = solver.eigenvalues().real();
        for(std::size_t i = 0 ; i < Dim ; ++i) {
            diagonal(i) = sqrt(diagonal(i));
        }
    }

    inline typename Types::Vector next() {
        typename Types::Vector sample;
        for(std::size_t i = 0 ; i < Dim ; ++i) {
            sample(i) = distribution(engine) * diagonal(i);
        }
        return rotation * sample + mean;
    }


private:
    typename  Types::RandomDevice         rd;
    typename  Types::GaussianDistribution distribution;
    typename  Types::RandomEngine         engine;

    typename  Types::Matrix rotation;
    typename  Types::Vector diagonal;
    typename  Types::Vector mean;

};

template<typename Scalar>
class Gaussian<Scalar, 1> {
public:
    typedef std::shared_ptr<Gaussian> Ptr;
    typedef Type<Scalar, 1>                       Types;

    Gaussian() :
        engine(rd())
    {
    }

    Gaussian(const Scalar _mean, const Scalar _std_dev) :
        distribution(_mean, _std_dev),
        engine(rd())
    {
    }

    void set(const Scalar _mean, const Scalar _std_dev)
    {
        distribution = typename Types::GaussianDistribution(_mean, _std_dev);
    }

    inline Scalar next()
    {
        return distribution(engine);
    }


private:
    typename  Types::RandomDevice         rd;
    typename  Types::GaussianDistribution distribution;
    typename  Types::RandomEngine         engine;
};

typedef Gaussian<double, 1> RNGGaussian;
typedef Gaussian<double, 3> RNGGaussian3D;
typedef Uniform<double, 1>  RNGUniform;
typedef Uniform<double, 3>  RNGUniform3D;


}
#endif // MATH_HPP

