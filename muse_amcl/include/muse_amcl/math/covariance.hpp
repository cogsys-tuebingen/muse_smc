#ifndef COVARIANCE_HPP
#define COVARIANCE_HPP

#include <eigen3/Eigen/Core>

namespace muse_amcl {
namespace math {
/**
 * @brief The Covariance class is a helper class for representing
 *        covariance matrices. It allows to isolate only the 3D
 *        components from the 6D representation.
 */
class Covariance {
public:
    using Matrix6d = Eigen::Matrix<double, 6, 6>;
    using Matrix3d = Eigen::Matrix3d;

    /** Covariance content 6D.
     (x     x)(x     y)(x     z)(x     phi)(x     theta)(x     psi)
     (y     x)(y     y)(y     z)(y     phi)(y     theta)(y     psi)
     (z     x)(z     y)(z     z)(z     phi)(z     theta)(z     psi)
     (phi   x)(phi   y)(phi   z)(phi   phi)(phi   theta)(phi   psi)
     (theta x)(theta y)(theta z)(theta phi)(theta theta)(theta psi)
     (psi   x)(psi   y)(psi   z)(psi   phi)(psi   theta)(psi   psi)
     */

    /** Covariance content 3D in compact representation.
     *  Data is stored that unused elements are set to zero.
     (x     x)(x     y)(x     z)(x     psi)
     (y     x)(y     y)(y     z)(y     psi)
     (psi   x)(psi   y)(psi   z)(psi   psi)
     */

    /**
     * @brief Covariance constructor for the full 3D covariance.
     * @param data - the covariance data
     */
    Covariance(const Matrix6d &data) :
        data_(data)
    {
    }

    /**
     * @brief Covariance constructor for the 2D covariance.
     * @param data
     */
    Covariance(const Matrix3d &data) :
        data_(Matrix6d::Zero())
    {
        data_.block<2,2>(0,0) = data.block<2,2>(0,0);
        data_(5,1) = data(2,0);
        data_(5,2) = data(2,1);
        data_(0,5) = data(0,2);
        data_(1,5) = data(1,2);
        data_(2,5) = data(2,2);
    }

    /**
     * @brief Covariance constructor with data initialization.
     * @param data - row major representation
     */
    Covariance(const std::vector<double> &data) :
        data_(Matrix6d::Zero())
    {
        auto get = [data](std::size_t r, std::size_t c, std::size_t step)
        {
            return data[r * step + c];
        };

        switch(data.size()) {
        case 9:
            data_(0,0) = get(0,0,3);
            data_(1,0) = get(1,0,3);
            data_(5,0) = get(2,0,3);
            data_(0,1) = get(0,1,3);
            data_(1,1) = get(1,1,3);
            data_(5,1) = get(0,2,3);
            data_(0,5) = get(0,1,3);
            data_(1,5) = get(1,1,3);
            data_(5,5) = get(0,2,3);
            break;
        case 36:
            for(std::size_t r = 0 ; r < 6 ; ++r) {
                for(std::size_t c = 0 ; c < 6 ; ++c) {
                    data_(r,c) = get(r,c,6);      /// for formatting
                }
            }
            break;
        default:
            throw std::runtime_error("[Covariance] :  data must have size 3 or 6.");
        }
    }

    /**
     * @brief data
     * @return const reference to data
     */
    inline const Matrix6d &data() const
    {
        return data_;
    }

    /**
     * @brief data return a reference to the data matrix.
     * @return reference to data
     */
    inline Matrix6d &data()
    {
        return data_;
    }

    /**
     * @brief as6D returns a 6D copy of the data. Otherwise the
     *        data() getter methods can be used.
     * @return copy of 6D matrix
     */
    inline Matrix6d eigen6D() const
    {
        return data_;
    }

    /**
     * @brief as3D returns the 3D components including correlated values
     *        with components x,y,psi
     * @return copy of 3D components
     */
    inline Matrix3d eigen3D() const
    {
        Matrix3d m;
        m.block<2,2>(0,0) = data_.block<2,2>(0,0);
        m(2,0) = data_(5,1);
        m(2,1) = data_(5,2);
        m(0,2) = data_(0,5);
        m(1,2) = data_(1,5);
        m(2,2) = data_(2,5);
        return m;
    }

    /**
     * @brief operator [] returns a reference to a data entry.
     * @param r - the row
     * @param c - the column
     * @return double reference
     */
    inline double& operator () (const std::size_t r, const std::size_t c)
    {
        return data_(r,c);
    }

    /**
     * @brief operator [] returns a const reference to a data entry.
     * @param r - the row
     * @param c - the column
     * @return const double reference
     */
    inline const double &operator () (const std::size_t r, const std::size_t c) const
    {
        return data_(r,c);
    }

private:
    Matrix6d data_;
};
}
}

#endif /* COVARIANCE_HPP */
