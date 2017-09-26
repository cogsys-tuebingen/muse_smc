#ifndef WEIGHTED_DISTRIBUTION_HPP
#define WEIGHTED_DISTRIBUTION_HPP

#undef NDEBUG
#include <assert.h>
#include <memory>
#include <mutex>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Eigen>
#include <iostream>

namespace muse_smc {
namespace math {
namespace statistic {
template<std::size_t Dim, bool limit_covariance = false>
class WeightedDistribution {
public:
    using Ptr                = std::shared_ptr<WeightedDistribution<Dim, limit_covariance>> ;
    using PointType          = Eigen::Matrix<double, Dim, 1>                                ;
    using MatrixType         = Eigen::Matrix<double, Dim, Dim>                              ;
    using EigenValueSetType  = Eigen::Matrix<double, Dim, 1>                                ;
    using EigenVectorSetType = Eigen::Matrix<double, Dim, Dim>                              ;
    using ComplexVectorType  = Eigen::Matrix<double, Dim, 1>                                ;
    using ComplexMatrixType  = Eigen::Matrix<std::complex<double>, Dim, Dim>                ;

    static constexpr double sqrt_2_M_PI = std::sqrt(2 * M_PI);
    static constexpr double lambda_ratio = 1e-2;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    WeightedDistribution() :
        sample_count_(0),
        mean_(PointType::Zero()),
        correlated_(MatrixType::Zero()),
        W_(0.0),
        W_1_(0.0),
        covariance_(MatrixType::Zero()),
        inverse_covariance_(MatrixType::Zero()),
        eigen_values_(EigenValueSetType::Zero()),
        eigen_vectors_(EigenVectorSetType::Zero()),
        determinant_(0.0),
        dirty_(false),
        dirty_eigen_(false)
    {
    }

    WeightedDistribution(const WeightedDistribution &other)             = default;
    WeightedDistribution& operator=(const WeightedDistribution &other)  = default;

    inline void reset()
    {
        mean_       = PointType::Zero();
        covariance_ = MatrixType::Zero();
        correlated_ = MatrixType::Zero();
        W_ = 1;
        W_1_ = 0;
        sample_count_ = 0;
        dirty_ = true;
        dirty_eigen_ = true;
    }

    /// Modification
    inline void add(const PointType &p, const double w)
    {
        W_  += w;
        mean_ = (mean_ * W_1_ + w * p) / W_;
        for(std::size_t i = 0 ; i < Dim ; ++i) {
            for(std::size_t j = i ; j < Dim ; ++j) {
                correlated_(i, j) = (correlated_(i, j) * W_1_ + w * p(i) * p(j)) / (double) W_;
            }
        }
        ++sample_count_;
        W_1_ = W_;
        dirty_ = true;
        dirty_eigen_ = true;
    }


    inline WeightedDistribution& operator+=(const WeightedDistribution &other)
    {
        mean_ = (mean_ * W_ + other.mean_ * other.W_) / (W_ + other.W_);
        correlated_ = (correlated_ * W_ +  other.correlated_ * other.W_) / (W_ + other.W_);
        W_ += other.W_;
        W_1_ = W_;
        sample_count_ += other.sample_count_;
        dirty_ = true;
        dirty_eigen_ = true;
        return *this;
    }

    /// Distribution properties
    inline std::size_t getSampleCount() const
    {
        return sample_count_;
    }

    inline double getWeight() const
    {
        return W_;
    }

    inline PointType getMean() const
    {
        return mean_;
    }

    inline void getMean(PointType &mean) const
    {
        mean = mean_;
    }

    inline MatrixType getCovariance() const
    {
        if(W_ > 0.0) {
            if(dirty_)
                update();
            return covariance_;
        }
        return MatrixType::Zero();
    }

    inline void getCovariance(MatrixType &covariance) const
    {
        if(W_ > 0.0) {
            if(dirty_)
                update();
            covariance = covariance_;
        } else {
            covariance = MatrixType::Zero();
        }
    }

    inline MatrixType getInformationMatrix() const
    {
        if(W_ > 0.0) {
            if(dirty_)
                update();
            return inverse_covariance_;
        }
        return MatrixType::Zero();
    }

    inline void getInformationMatrix(MatrixType &inverse_covariance) const
    {
        if(W_ > 0.0) {
            if(dirty_)
                update();
            inverse_covariance = inverse_covariance_;
        } else {
            inverse_covariance = MatrixType::Zero();
        }
    }

    inline EigenValueSetType getEigenValues(const bool abs = false) const
    {
        if(W_ > 0.0) {
            if(dirty_)
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
        if(W_ > 0.0) {
            if(dirty_)
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
        if(W_ > 0.0) {
            if(dirty_)
                update();
            if(dirty_eigen_)
                updateEigen();

            return eigen_vectors_;
        }
        return EigenVectorSetType::Zero();
    }

    inline void getEigenVectors(EigenVectorSetType &eigen_vectors) const
    {
        if(W_ > 0.0) {
            if(dirty_)
                update();
            if(dirty_eigen_)
                updateEigen();

            eigen_vectors = eigen_vectors_;
        } else {
            eigen_vectors = EigenVectorSetType::Zero();
        }
    }

    /// Evaluation
    inline double sample(const PointType &p) const
    {
        if(W_ > 0.0) {
            if(dirty_)
                update();
            PointType  q = p - mean_;
            double exponent = -0.5 * double(q.transpose() * inverse_covariance_ * q);
            double denominator = 1.0 / (covariance_.determinant() * sqrt_2_M_PI);
            return denominator * std::exp(exponent);
        }
        return 0.0;
    }

    inline double sample(const PointType &p,
                         PointType &q) const
    {
        if(W_ > 0.0) {
            if(dirty_)
                update();
            q = p - mean_;
            double exponent = -0.5 * double(q.transpose() * inverse_covariance_ * q);
            double denominator = 1.0 / (determinant_ * sqrt_2_M_PI);
            return denominator * std::exp(exponent);
        }
        return 0.0;
    }

    inline double sampleNonNormalized(const PointType &p) const
    {
        if(W_ > 0.0) {
            if(dirty_)
                update();

            PointType  q = p - mean_;
            double exponent = -0.5 * double(q.transpose() * inverse_covariance_ * q);
            return std::exp(exponent);
        }
        return 0.0;
    }

    inline double sampleNonNormalized(const PointType &p,
                                      PointType &q) const
    {
        if(W_1_ >= 2) {
            if(dirty_)
                update();
            q = p - mean_;
            double exponent = -0.5 * double(q.transpose() * inverse_covariance_ * q);
            return std::exp(exponent);
        }
        return 0.0;
    }

private:
    std::size_t                  sample_count_;
    PointType                    mean_;
    MatrixType                   correlated_;
    double                       W_;
    double                       W_1_;            /// actual amount of points in distribution

    mutable MatrixType           covariance_;
    mutable MatrixType           inverse_covariance_;
    mutable EigenValueSetType    eigen_values_;
    mutable EigenVectorSetType   eigen_vectors_;
    mutable double               determinant_;

    mutable bool                 dirty_;
    mutable bool                 dirty_eigen_;

    inline void update() const
    {
        for(std::size_t i = 0 ; i < Dim ; ++i) {
            for(std::size_t j = i ; j < Dim ; ++j) {
                covariance_(i, j) = (correlated_(i, j) - (mean_(i) * mean_(j))) / W_;
                covariance_(j, i) = covariance_(i, j);
            }
        }

        if(limit_covariance) {
            if(dirty_eigen_)
                updateEigen();

            double max_lambda = std::numeric_limits<double>::lowest();
            for(std::size_t i = 0 ; i < Dim ; ++i) {
                if(eigen_values_(i) > max_lambda)
                    max_lambda = eigen_values_(i);
            }
            MatrixType Lambda = MatrixType::Zero();
            double l = max_lambda * lambda_ratio;
            for(std::size_t i = 0 ; i < Dim; ++i) {
                if(fabs(eigen_values_(i)) < fabs(l)) {
                    Lambda(i,i) = l;
                } else {
                    Lambda(i,i) = eigen_values_(i);
                }
            }
            covariance_ = eigen_vectors_ * Lambda * eigen_vectors_.transpose();
            inverse_covariance_ = eigen_vectors_ * Lambda.inverse() * eigen_vectors_.transpose();
        } else {
            inverse_covariance_ = covariance_.inverse();
        }

        determinant_ = covariance_.determinant();
        dirty_ = false;
        dirty_eigen_ = true;
    }

    inline void updateEigen() const
    {
        Eigen::EigenSolver<MatrixType> solver;
        solver.compute(covariance_);
        eigen_vectors_ = solver.eigenvectors().real();
        eigen_values_  = solver.eigenvalues().real();
        dirty_eigen_ = false;
    }
};
}
}
}

#endif // WEIGHTED_DISTRIBUTION_HPP
