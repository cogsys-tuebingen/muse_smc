#ifndef WEIGHTED_DISTRIBUTION_HPP
#define WEIGHTED_DISTRIBUTION_HPP

#undef NDEBUG
#include <assert.h>
#include <memory>
#include <mutex>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Eigen>
#include <iostream>

namespace muse_amcl {
namespace math {
namespace statistic {
template<std::size_t Dim, bool limit_covariance = false>
class WeightedDistribution {
public:
    typedef std::shared_ptr<WeightedDistribution<Dim, limit_covariance>> Ptr;

    typedef Eigen::Matrix<double, Dim, 1>                        PointType;
    typedef Eigen::Matrix<double, Dim, Dim>                      MatrixType;
    typedef Eigen::Matrix<double, Dim, 1>                        EigenValueSetType;
    typedef Eigen::Matrix<double, Dim, Dim>                      EigenVectorSetType;
    typedef Eigen::Matrix<double, Dim, 1>                        ComplexVectorType;
    typedef Eigen::Matrix<std::complex<double>, Dim, Dim>        ComplexMatrixType;

    static constexpr double sqrt_2_M_PI = std::sqrt(2 * M_PI);
    static constexpr double lambda_ratio = 1e-2;

    WeightedDistribution() :
        mean(PointType::Zero()),
        correlated(MatrixType::Zero()),
        W(0.0),
        W_1(0.0),
        covariance(MatrixType::Zero()),
        inverse_covariance(MatrixType::Zero()),
        eigen_values(EigenValueSetType::Zero()),
        eigen_vectors(EigenVectorSetType::Zero()),
        determinant(0.0),
        dirty(false),
        dirty_eigen(false)
    {
    }

    WeightedDistribution(const WeightedDistribution &other) = default;
    WeightedDistribution& operator=(const WeightedDistribution &other) = default;

    inline void reset()
    {
        mean       = PointType::Zero();
        covariance = MatrixType::Zero();
        correlated = MatrixType::Zero();
        W = 1;
        W_1 = 0;
        dirty = true;
        dirty_eigen = true;
    }

    /// Modification
    inline void add(const PointType &_p, const double w)
    {
        W  += w;
        mean = (mean * W_1 + w * _p) / W;
        for(std::size_t i = 0 ; i < Dim ; ++i) {
            for(std::size_t j = i ; j < Dim ; ++j) {
                correlated(i, j) = (correlated(i, j) * W_1 + w * _p(i) * _p(j)) / (double) W;
            }
        }
        W_1 = W;
        dirty = true;
        dirty_eigen = true;
    }


    inline WeightedDistribution& operator+=(const WeightedDistribution &other)
    {
        mean = (mean * W + other.mean * other.W) / (W + other.W);
        correlated = (correlated * W +  other.correlated * other.W) / (W + other.W);
        W += other.W;
        W_1 = W;
        dirty = true;
        dirty_eigen = true;
        return *this;
    }

    /// Distribution properties
    inline double getWeight() const
    {
        return W;
    }

    inline PointType getMean() const
    {
        return mean;
    }

    inline void getMean(PointType &_mean) const
    {
        _mean = mean;
    }

    inline MatrixType getCovariance() const
    {
        if(W > 0.0) {
            if(dirty)
                update();
            return covariance;
        }
        return MatrixType::Zero();
    }

    inline void getCovariance(MatrixType &_covariance) const
    {
        if(W > 0.0) {
            if(dirty)
                update();
            _covariance = covariance;
        } else {
            _covariance = MatrixType::Zero();
        }
    }

    inline MatrixType getInformationMatrix() const
    {
        if(W > 0.0) {
            if(dirty)
                update();
            return inverse_covariance;
        }
        return MatrixType::Zero();
    }

    inline void getInformationMatrix(MatrixType &_inverse_covariance) const
    {
        if(W > 0.0) {
            if(dirty)
                update();
            _inverse_covariance = inverse_covariance;
        } else {
            _inverse_covariance = MatrixType::Zero();
        }
    }

    inline EigenValueSetType getEigenValues(const bool _abs = false) const
    {
        if(W > 0.0) {
            if(dirty)
                update();
            if(dirty_eigen)
                updateEigen();

            if(_abs)
                return eigen_values.cwiseAbs();
            else
                return eigen_values;
        }
        return EigenValueSetType::Zero();
    }

    inline void getEigenValues(EigenValueSetType &_eigen_values,
                               const double _abs = false) const
    {
        if(W > 0.0) {
            if(dirty)
                update();
            if(dirty_eigen)
                updateEigen();

            if(_abs)
                _eigen_values = eigen_values.cwiseAbs();
            else
                _eigen_values = eigen_values;
        } else {
            _eigen_values = EigenValueSetType::Zero();
        }
    }

    inline EigenVectorSetType getEigenVectors() const
    {
        if(W > 0.0) {
            if(dirty)
                update();
            if(dirty_eigen)
                updateEigen();

            return eigen_vectors;
        }
        return EigenVectorSetType::Zero();
    }

    inline void getEigenVectors(EigenVectorSetType &_eigen_vectors) const
    {
        if(W > 0.0) {
            if(dirty)
                update();
            if(dirty_eigen)
                updateEigen();

            _eigen_vectors = eigen_vectors;
        } else {
            _eigen_vectors = EigenVectorSetType::Zero();
        }
    }

    /// Evaluation
    inline double sample(const PointType &_p) const
    {
        if(W > 0.0) {
            if(dirty)
                update();
            PointType  q = _p - mean;
            double exponent = -0.5 * double(q.transpose() * inverse_covariance * q);
            double denominator = 1.0 / (covariance.determinant() * sqrt_2_M_PI);
            return denominator * exp(exponent);
        }
        return 0.0;
    }

    inline double sample(const PointType &_p,
                         PointType &_q) const
    {
        if(W > 0.0) {
            if(dirty)
                update();
            _q = _p - mean;
            double exponent = -0.5 * double(_q.transpose() * inverse_covariance * _q);
            double denominator = 1.0 / (determinant * sqrt_2_M_PI);
            return denominator * exp(exponent);
        }
        return 0.0;
    }

    inline double sampleNonNormalized(const PointType &_p) const
    {
        if(W > 0.0) {
            if(dirty)
                update();

            PointType  q = _p - mean;
            double exponent = -0.5 * double(q.transpose() * inverse_covariance * q);
            return exp(exponent);
        }
        return 0.0;
    }

    inline double sampleNonNormalized(const PointType &_p,
                                      PointType &_q) const
    {
        if(W_1 >= 2) {
            if(dirty)
                update();
            _q = _p - mean;
            double exponent = -0.5 * double(_q.transpose() * inverse_covariance * _q);
            return exp(exponent);
        }
        return 0.0;
    }

private:
    PointType                    mean;
    MatrixType                   correlated;
    double                       W;
    double                       W_1;            /// actual amount of points in distribution

    mutable MatrixType           covariance;
    mutable MatrixType           inverse_covariance;
    mutable EigenValueSetType    eigen_values;
    mutable EigenVectorSetType   eigen_vectors;
    mutable double               determinant;

    mutable bool                 dirty;
    mutable bool                 dirty_eigen;

    inline void update() const
    {
        for(std::size_t i = 0 ; i < Dim ; ++i) {
            for(std::size_t j = i ; j < Dim ; ++j) {
                covariance(i, j) = (correlated(i, j) - (mean(i) * mean(j))) / W;
                covariance(j, i) = covariance(i, j);
            }
        }

        if(limit_covariance) {
            if(dirty_eigen)
                updateEigen();

            double max_lambda = std::numeric_limits<double>::lowest();
            for(std::size_t i = 0 ; i < Dim ; ++i) {
                if(eigen_values(i) > max_lambda)
                    max_lambda = eigen_values(i);
            }
            MatrixType Lambda = MatrixType::Zero();
            double l = max_lambda * lambda_ratio;
            for(std::size_t i = 0 ; i < Dim; ++i) {
                if(fabs(eigen_values(i)) < fabs(l)) {
                    Lambda(i,i) = l;
                } else {
                    Lambda(i,i) = eigen_values(i);
                }
            }
            covariance = eigen_vectors * Lambda * eigen_vectors.transpose();
            inverse_covariance = eigen_vectors * Lambda.inverse() * eigen_vectors.transpose();
        } else {
            inverse_covariance = covariance.inverse();
        }

        determinant = covariance.determinant();
        dirty = false;
        dirty_eigen = true;
    }

    inline void updateEigen() const
    {
        Eigen::EigenSolver<MatrixType> solver;
        solver.compute(covariance);
        eigen_vectors = solver.eigenvectors().real();
        eigen_values  = solver.eigenvalues().real();
        dirty_eigen = false;
    }
};
}
}
}


#endif // WEIGHTED_DISTRIBUTION_HPP
