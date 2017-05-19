#ifndef MLE_PARAMETER_ESTIMATOR_HPP
#define MLE_PARAMETER_ESTIMATOR_HPP

#include <vector>

class MLEParameterEstimator {
public:
    MLEParameterEstimator() :
        measurements_(0ul),
        z_hit_(0.0),
        z_short_(0.0),
        z_max_(0.0),
        z_rand_(0.0),
        sigma_hit(0.0),
        lambda_short_(0.0)
    {
    }

    inline void insert(const double z,
                       const double z_bar)
    {
        p_hit_.emplace_back(p_hit);
        p_short_.emplace_back(p_short);
        p_rand_.emplace_back(p_max);
        p_max_.emplace_back(p_rand);
        z_.emplace_back(z);
        z_bar_.emplace_back(z_bar);
        ++measurements_;
    }

    inline void estimate()
    {
        const std::size_t measurement_count = p_hit_.size();

        double sum_e_hit     = 0.0;
        double sum_e_short   = 0.0;
        double sum_e_max     = 0.0;
        double sum_e_rand    = 0.0;
        double sum_e_short_z = 0.0;

        double sum_diff = 0.0;

        auto sq = [](const double a){return a*a;};

        for(std::size_t i = 0 ; i < measurement_count ; ++i) {
            const double p_hit   = p_hit_[i];
            const double p_max = p_max_[i];
            const double p_short = p_short_[i];
            const double p_rand  = p_rand_[i];
            const double nu = 1.0 / (p_hit + p_rand + p_short + p_max);
            const double e_hit   = nu * p_hit;
            const double e_short = nu * p_short;
            const double e_max   = nu * p_max;
            const double e_rand  = nu * p_rand;
            sum_e_hit     += e_hit;
            sum_e_short   += e_short;
            sum_e_max     += e_max;
            sum_e_rand    += e_rand;
            sum_e_short_z += e_short * z_[i];
            sum_diff += e_hit * sq(z_[i] - z_bar_[i]);
        }


    }


private:
    std::size_t measurements_;

    double z_hit_;
    double z_short_;
    double z_max_;
    double z_rand_;
    double sigma_hit_;
    double lambda_short_;

    std::vector<double> p_hit_;
    std::vector<double> p_short_;
    std::vector<double> p_rand_;
    std::vector<double> p_max_;
    std::vector<double> z_;
    std::vector<double> z_bar_;


}

#endif // MLE_PARAMETER_ESTIMATOR_HPP
