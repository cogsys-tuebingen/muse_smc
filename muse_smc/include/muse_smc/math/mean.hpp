#ifndef MEAN_HPP
#define MEAN_HPP

#include <eigen3/Eigen/Core>

namespace muse_smc {
namespace math {
namespace statistic {
template<std::size_t Dim>
class Mean
{
public:
    using sample_t = Eigen::Matrix<double, Dim, 1>;

    Mean() :
        mean_(0.0),
        n_(1),
        n_1(0)
    {
    }

    inline void add(const sample_t &sample)
    {
        mean_ = (mean_ * n_1 + sample) / n_;
        ++n_;
        ++n_1;
    }

    inline double get() const
    {
        return mean_;
    }

private:
    sample_t    mean_;
    std::size_t n_;
    std::size_t n_1;
};

template<>
class Mean<1>
{
public:
    Mean() :
        mean_(0.0),
        n_(1),
        n_1(0)
    {
    }

    inline void add(const double &sample)
    {
        mean_ = (mean_ * n_1 + sample) / n_;
        ++n_;
        ++n_1;
    }

    inline double get() const
    {
        return mean_;
    }

private:
    double mean_;
    std::size_t n_;
    std::size_t n_1;

};
}
}
}

#endif // MEAN_HPP
