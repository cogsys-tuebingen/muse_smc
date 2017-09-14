#ifndef ANGULAR_MEAN_HPP
#define ANGULAR_MEAN_HPP

#include <memory>
#include <muse_smc/math/angle.hpp>

namespace muse_smc {
namespace math {
namespace statistic {
class AngularMean {
public:
    using Ptr = std::shared_ptr<AngularMean>;
    using complex = std::complex<double>;

    AngularMean() :
        dirty_(true),
        mean_(0.0),
        complex_mean_(0.0, 0.0),
        n_(1ul),
        n_1_(0ul)
    {
    }

    AngularMean(const AngularMean &other) :
        dirty_(true),
        mean_(other.mean_),
        complex_mean_(other.complex_mean_),
        n_(other.n_),
        n_1_(other.n_1_)
    {
    }

    AngularMean(AngularMean &&other) :
        dirty_(true),
        mean_(other.mean_),
        complex_mean_(std::move(other.complex_mean_)),
        n_(other.n_),
        n_1_(other.n_1_)
    {
    }

    AngularMean& operator=(const AngularMean &other)
    {
        dirty_          = true;
        mean_           = other.mean_;
        complex_mean_   = other.complex_mean_;
        n_              = other.n_;
        n_1_            = other.n_1_;
        return *this;
    }

    AngularMean& operator=(AngularMean &&other)
    {
        dirty_          = true;
        mean_           = other.mean_;
        complex_mean_   = std::move(other.complex_mean_);
        n_              = other.n_;
        n_1_            = other.n_1_;
        return *this;
    }

    void reset()
    {
        dirty_ = true;
        mean_ = 0.0;
        complex_mean_ = 0.0;
        n_ = 1ul;
        n_1_ = 0ul;
    }

    inline void add(const double rad)
    {
        complex_mean_ = (complex_mean_ * static_cast<double>(n_1_) + angle::toComplex(rad)) /
                         static_cast<double>(n_);
        ++n_;
        ++n_1_;
        dirty_ = true;
    }

    inline AngularMean& operator += (const AngularMean& other)
    {
        dirty_ = true;
        std::size_t _n = n_1_ + other.n_1_;
        complex   _mean = (complex_mean_ * static_cast<double>(n_1_) + other.complex_mean_ * static_cast<double>(other.n_1_)) /
                           static_cast<double>(_n);
        complex_mean_ = _mean;
        n_   = _n + 1;
        n_1_ = _n;
        dirty_ = true;
        return *this;
    }

    inline double getN() const
    {
        return n_1_;
    }

    inline double getMean() const
    {
        if(dirty_) {
            mean_ = angle::fromComplex(complex_mean_);
            dirty_ = false;
        }
        return mean_;
    }

    inline void getMean(double &mean) {
        if(dirty_) {
            mean_ = angle::fromComplex(complex_mean_);
            dirty_ = false;
        }
        mean = mean_;
    }

    inline double getCovariance() const
    {
        return -2.0 * std::log(std::hypot(complex_mean_.real(), complex_mean_.imag()));
    }

private:
    mutable bool    dirty_;
    mutable double  mean_;
    complex complex_mean_;
    std::size_t n_;
    std::size_t n_1_;
}__attribute__ ((aligned (64)));
}
}
}
#endif // ANGULAR_MEAN_HPP
