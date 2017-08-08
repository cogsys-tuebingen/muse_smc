#ifndef COVARIANCE_2D_HPP
#define COVARIANCE_2D_HPP

#include <array>
#include <Eigen/Core>


namespace muse_mcl_2d {
class Covariance2D {
public:
    using data_t = std::array<double, 9>;

    Covariance2D()
    {
        data_.fill(0.0);
    }

    Covariance2D(const data_t &data) :
        data_(data)
    {
    }

    inline double & operator () (const std::size_t i, const std::size_t j)
    {
        return data_[size_ * i + j];
    }

    inline double const & operator () (const std::size_t i, const std::size_t j) const
    {
        return data_[size_ * i + j];
    }

    inline void fromEigen(const Eigen::Matrix3d &e)
    {
        for(std::size_t i = 0 ; i < size_ ; ++i) {
            for(std::size_t j = 0 ; j < size_ ; ++j) {
                 data_[size_ * i + j] = e(i,j);
            }
        }
    }

    inline Eigen::Matrix3d toEigen() const
    {
        Eigen::Matrix3d e;
        for(std::size_t i = 0 ; i < size_ ; ++i) {
            for(std::size_t j = 0 ; j < size_ ; ++j) {
                e(i,j) = data_[size_ * i + j];
            }
        }
        return e;
    }

    inline void toEigen(Eigen::Matrix3d &e) const
    {
        for(std::size_t i = 0 ; i < size_ ; ++i) {
            for(std::size_t j = 0 ; j < size_ ; ++j) {
                e(i,j) = data_[size_ * i + j];
            }
        }
    }

private:
    const static std::size_t size_ = 3;
    data_t data_;


};
}

#endif // COVARIANCE_2D_HPP
