#ifndef ANGULAR_DISTRIBUTION_HPP
#define ANGULAR_DISTRIBUTION_HPP

#undef NDEBUG
#include <assert.h>
#include <memory>
#include <mutex>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Eigen>
#include <iostream>

#include <muse_mcl/math/angle.hpp>


namespace muse_mcl {
namespace math {
namespace statistic {
template<std::size_t Dim>
class AngularDistribution {
public:
    using Ptr = std::shared_ptr<AngularDistribution<Dim>>;
    using VectorType         = Eigen::Matrix<double, Dim, 1>;
    using MatrixType         = Eigen::Matrix<double, Dim, Dim>;

    using EigenValueSetType  = Eigen::Matrix<double, Dim, 1>;
    using EigenVectorSetType = Eigen::Matrix<double, Dim, Dim>;

    using complex            = std::complex<double>;
    using ComplexVectorType  = Eigen::Matrix<complex, Dim, 1>;
    using ComplexMatrixType  = Eigen::Matrix<complex, Dim, Dim>;

    static constexpr double sqrt_2_M_PI = std::sqrt(2 * M_PI);

    AngularDistribution() :
        mean_complex_(ComplexVectorType::Zero()),
        correlated_complex_(ComplexMatrixType::Zero()),
        covariance_complex_(ComplexMatrixType::Zero()),
        inverse_covariance_complex_(ComplexMatrixType::Zero()),
        eigen_values_complex_(ComplexVectorType::Zero()),
        eigen_vectors_complex_(ComplexMatrixType::Zero()),
        determinant_(complex()),
        mean_(VectorType::Zero()),
        covariance_(MatrixType::Zero()),
        inverse_covariance_(MatrixType::Zero()),
        eigen_values_(EigenValueSetType::Zero()),
        eigen_vectors_(EigenVectorSetType::Zero()),
        n_(1),
        n_1_(0),
        dirty_covariance_(false),
        dirty_eigen_(false),
        dirty_mean_(false)
    {
    }

    AngularDistribution(const AngularDistribution &other) = default;
    AngularDistribution& operator=(const AngularDistribution &other) = default;

    inline void reset()
    {
        mean_               = VectorType::Zero();
        covariance_         = MatrixType::Zero();
        correlated_complex_ = ComplexMatrixType::Zero();
        n_ = 1;
        n_1_ = 0;
        dirty_covariance_ = true;
        dirty_eigen_ = true;
    }

    inline void add(const VectorType &p)
    {
        auto pc = toComplex(p);

        mean_complex_ = (mean_complex_ * static_cast<complex>(n_1_) + pc) / static_cast<complex>(n_);
        for(std::size_t i = 0 ; i < Dim ; ++i) {
            for(std::size_t j = i ; j < Dim ; ++j) {
                correlated_complex_(i, j) =
                        (correlated_complex_(i, j) * static_cast<complex>(n_1_) + pc(i) * pc(j)) /  static_cast<complex>(n_);
            }
        }
        ++n_  ;
        ++n_1_;
        dirty_covariance_ = true;
        dirty_eigen_      = true;
        dirty_mean_       = true;
    }

    inline AngularDistribution& operator+=(const VectorType &p)
    {
        add(p);
        return *this;
    }


    inline AngularDistribution& operator+=(const AngularDistribution &other)
    {
        std::size_t _n = n_1_ + other.n_1_;
        ComplexVectorType  _mean_complex = (mean_complex_ * static_cast<complex>(n_1_) +
                                     other.mean_complex_ * static_cast<complex>(other.n_1_)) / static_cast<complex>(_n);
        ComplexMatrixType  _corr_complex =
                (correlated_complex_ * static_cast<complex>(n_1_) + other.correlated_complex_ * static_cast<complex>(other.n_1_)) / _n;
        n_                  = _n + 1;
        n_1_                = _n;
        mean_complex_       = _mean_complex;
        correlated_complex_ = _corr_complex;
        dirty_covariance_ = true;
        dirty_eigen_      = true;
        dirty_mean_       = true;
        return *this;
    }

    /// Distribution properties
    inline std::size_t getN() const
    {
        return n_1_;
    }

    inline VectorType getMean() const
    {
        if(dirty_mean_) {
            mean_ = fromComplex(mean_complex_);
            dirty_mean_ = false;
        }

        return mean_;
    }

    inline void getMean(VectorType &_mean) const
    {
        if(dirty_mean_) {
            mean_ = fromComplex(mean_complex_);
            dirty_mean_ = false;
        }

        _mean = mean_;
    }

    inline MatrixType getCovariance() const
    {
        if(n_1_ >= 2) {
            if(dirty_covariance_)
                update();
            return covariance_;
        }
        return MatrixType::Zero();
    }

    inline void getCovariance(MatrixType &covariance) const
    {
        if(n_1_ >= 2) {
            if(dirty_covariance_)
                update();
            covariance = covariance_;
        } else {
            covariance = MatrixType::Zero();
        }
    }

    inline MatrixType getInformationMatrix() const
    {
        if(n_1_ >= 2) {
            if(dirty_covariance_)
                update();
            return inverse_covariance_;
        }
        return MatrixType::Zero();
    }

    inline void getInformationMatrix(MatrixType &inverse_covariance) const
    {
        if(n_1_ >= 2) {
            if(dirty_covariance_)
                update();
            inverse_covariance = inverse_covariance_;
        } else {
            inverse_covariance = MatrixType::Zero();
        }
    }

    inline EigenValueSetType getEigenValues(const bool abs = false) const
    {
        if(n_1_ >= 2) {
            if(dirty_covariance_)
                update();
            if(dirty_eigen_)
                updateEigen();

            if(abs)
                return eigen_values_.cwiseAbs();
            else
                return eigen_values_;
        }
        return EigenValueSetType::Zero();
    }

    inline void getEigenValues(EigenValueSetType &eigen_values,
                               const double abs = false) const
    {
        if(n_1_ >= 2) {
            if(dirty_covariance_)
                update();
            if(dirty_eigen_)
                updateEigen();

            if(abs)
                eigen_values = eigen_values_.cwiseAbs();
            else
                eigen_values = eigen_values_;
        } else {
            eigen_values = EigenValueSetType::Zero();
        }
    }

    inline EigenVectorSetType getEigenVectors() const
    {
        if(n_1_ >= 2) {
            if(dirty_covariance_)
                update();
            if(dirty_eigen_)
                updateEigen();

            return eigen_vectors_;
        }
        return EigenVectorSetType::Zero();
    }

    inline void getEigenVectors(EigenVectorSetType &eigen_vectors) const
    {
        if(n_1_ >= 2) {
            if(dirty_covariance_)
                update();
            if(dirty_eigen_)
                updateEigen();

            eigen_vectors = eigen_vectors_;
        } else {
            eigen_vectors = EigenVectorSetType::Zero();
        }
    }


    /// Evaluation
    inline double sample(const VectorType &p) const
    {
        if(n_1_ >= 2) {
            if(dirty_covariance_)
                update();
            ComplexVectorType  q = toComplex(p) - mean_complex_;
            double exponent    = -0.5 * complex(q.transpose() * inverse_covariance_ * q);
            double denominator = 1.0 / (determinant_ * sqrt_2_M_PI);
            return angle::fromComplex(denominator * std::exp(exponent));
        }
        return 0.0;
    }

    inline double sample(const VectorType &p,
                         VectorType &q) const
    {
        if(n_1_ >= 2) {
            if(dirty_covariance_)
                update();
            ComplexVectorType qc = p - mean_;
            q = fromComplex(qc);
            double exponent = -0.5 * complex(qc.transpose() * inverse_covariance_ * qc);
            double denominator = 1.0 / (determinant_ * sqrt_2_M_PI);
            return angle::fromComplex(denominator * std::exp(exponent));
        }
        return 0.0;
    }

    inline double sampleNonNormalized(const VectorType &p) const
    {
        if(n_1_ >= 2) {
            if(dirty_covariance_)
                update();

            VectorType  q = p - mean_;
            double exponent = -0.5 * double(q.transpose() * inverse_covariance_ * q);
            return angle::fromComplex(std::exp(exponent));
        }
        return 0.0;
    }

    inline double sampleNonNormalized(const VectorType &p,
                                      VectorType &q) const
    {
        if(n_1_ >= 2) {
            if(dirty_covariance_)
                update();
            ComplexVectorType qc = p - mean_;
            q = fromComplex(qc);
            double exponent = -0.5 * complex(qc.transpose() * inverse_covariance_ * qc);
            return angle::fromComplex(std::exp(exponent));
        }
        return 0.0;
    }

private:
    ComplexVectorType         mean_complex_;
    ComplexMatrixType         correlated_complex_;
    mutable ComplexMatrixType covariance_complex_;
    mutable ComplexMatrixType inverse_covariance_complex_;
    mutable ComplexVectorType eigen_values_complex_;
    mutable ComplexMatrixType eigen_vectors_complex_;
    mutable complex           determinant_;

    mutable VectorType        mean_;
    mutable MatrixType        covariance_;
    mutable MatrixType        inverse_covariance_;
    mutable VectorType        eigen_values_;
    mutable MatrixType        eigen_vectors_;

    std::size_t               n_;
    std::size_t               n_1_;

    mutable bool              dirty_covariance_;
    mutable bool              dirty_eigen_;
    mutable bool              dirty_mean_;


    inline void update() const
    {
        complex scale = static_cast<complex>(n_1_) / (static_cast<complex>(n_1_) - complex(1));
        for(std::size_t i = 0 ; i < Dim ; ++i) {
            for(std::size_t j = i ; j < Dim ; ++j) {
                covariance_complex_(i, j) = (correlated_complex_(i, j) - (mean_complex_(i) * mean_complex_(j))) * scale;
                covariance_complex_(j, i) = covariance_complex_(i, j);

                covariance_(i, j) = (angle::fromComplex(correlated_complex_(i, j)) -
                                             angle::fromComplex(mean_complex_(i) * mean_complex_(j))) / (n_1_ / (n_1_ - 1));
                covariance_(j, i) = covariance_(i, j);


            }
        }

        inverse_covariance_complex_ = covariance_complex_.inverse();
//        covariance_                 = fromComplexCovariance(covariance_complex_);
//        inverse_covariance_         = fromComplexCovariance(inverse_covariance_complex_);

        determinant_                = covariance_.determinant();
        dirty_covariance_           = false;
        dirty_eigen_                = true;
    }

    inline void updateEigen() const
    {
        Eigen::EigenSolver<ComplexMatrixType> solver;
        solver.compute(covariance_);
        eigen_vectors_complex_ = solver.eigenvectors();
        eigen_values_complex_  = solver.eigenvalues();
        eigen_vectors_         = fromComplex(eigen_vectors_complex_);
        eigen_values_          = fromComplex(eigen_values_complex_);
        dirty_eigen_           = false;
    }

    inline ComplexVectorType toComplex(const VectorType &v) const
    {
        ComplexVectorType vc;
        for(std::size_t i = 0 ; i < Dim ; ++i) {
            vc(i) = angle::toComplex(v(i));
        }
        return vc;
    }

    inline VectorType fromComplex(const ComplexVectorType &vc) const
    {
        VectorType v;
        for(std::size_t i = 0 ; i < Dim ; ++i) {
            v(i) = angle::fromComplex(vc(i));
        }
        return v;
    }

    inline MatrixType fromComplex(const ComplexMatrixType &mc) const
    {
        MatrixType m;
        for(std::size_t i = 0 ; i < Dim ; ++i) {
            for(std::size_t j = 0 ; j < Dim ; ++j) {
                m(i,j) = angle::fromComplex(mc(i,j));
            }
        }
        return m;
    }

    inline MatrixType fromComplexCovariance(const ComplexMatrixType &mc) const
    {
        MatrixType m;
        for(std::size_t i = 0 ; i < Dim ; ++i) {
            for(std::size_t j = i ; j < Dim ; ++j) {
                m(i,j) = angle::fromComplex(mc(i,j));
                m(j,i) = m(i,j);
            }
        }
        return m;
    }
};
}
}
}
#endif // ANGULAR_DISTRIBUTION_HPP
