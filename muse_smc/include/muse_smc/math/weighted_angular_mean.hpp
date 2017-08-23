#ifndef WEIGHTED_ANGULAR_MEAN_HPP
#define WEIGHTED_ANGULAR_MEAN_HPP

#include <memory>
#include <complex>
#include <muse_smc/math/angle.hpp>

namespace muse_smc {
namespace math {
namespace statistic {
class WeightedAngularMean {
public:
    using Ptr = std::shared_ptr<WeightedAngularMean>;
    using complex = std::complex<double>;

    WeightedAngularMean() :
        dirty_(true),
        mean_(0.0),
        complex_mean_(0.0, 0.0),
        W_(1.0),
        W_1_(0.0)
    {
    }

    WeightedAngularMean(const WeightedAngularMean &other) :
        dirty_(other.dirty_),
        mean_(other.mean_),
        complex_mean_(other.complex_mean_),
        W_(other.W_),
        W_1_(other.W_1_)
    {
    }

    WeightedAngularMean& operator=(const WeightedAngularMean &other)
    {
        if(this != &other) {
            dirty_ = other.dirty_;
            mean_  = other.mean_;
            complex_mean_ = other.complex_mean_;
            W_ = other.W_;
            W_1_ = other.W_1_;
        }
        return *this;
    }

    WeightedAngularMean& operator=(WeightedAngularMean &&other)
    {
        if(this != &other) {
            dirty_ = other.dirty_;
            mean_  = other.mean_;
            complex_mean_ = other.complex_mean_;
            W_ = other.W_;
            W_1_ = other.W_1_;
        }
        return *this;
    }

    void reset()
    {
        dirty_ = true;
        mean_ = 0.0;
        complex_mean_ = 0.0;
        W_ = 1.0;
        W_1_ = 0.0;
    }

    inline void add(const double rad, const double w)
    {
        W_  += w;
        complex_mean_ = (complex_mean_ * W_1_ + angle::toComplex(rad) * w) / W_;
        W_1_ = W_;
        dirty_ = true;
    }

    inline WeightedAngularMean& operator += (const WeightedAngularMean& other)
    {
        dirty_ = true;
        complex_mean_ = (complex_mean_ * W_ + other.complex_mean_ * other.W_) / (W_ + other.W_);
        W_ += other.W_;
        W_1_ = W_;
        return *this;
    }

    inline double getWeight() const
    {
        return W_;
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
    double  W_;
    double  W_1_;
};
}
}
}
#endif // WEIGHTED_ANGULAR_MEAN_HPP
