#include <cslibs_math/random/random.hpp>

#include <muse_mcl_2d/resampling/resampling_2d.hpp>
#include <muse_mcl_2d/density/sample_density_2d.hpp>

namespace muse_mcl_2d {
class LocalRegenerationKLD2D : public Resampling2D
{
public:

protected:
    double                       kld_error_;
    double                       kld_z_;
    cslibs_math_2d::Covariance2d covariance_;

    virtual void doSetup(ros::NodeHandle &nh) override
    {
        auto param_name = [this](const std::string &name){return name_ + "/" + name;};
        kld_error_ = nh.param(param_name("kld_error"), 0.01);
        kld_z_     = nh.param(param_name("kld_z"), 0.99);

        std::vector<double> c_v;
        nh.getParam(param_name("covariance"),c_v);

        if(c_v.size() != 9) {
            ROS_ERROR_STREAM("The initialization covariance is expected to have 9 values.");
            return;
        }

        auto get = [&c_v](std::size_t r, std::size_t c, std::size_t step)
        {
            return c_v[r * step + c];
        };

        covariance_(0,0) = get(0,0,3);
        covariance_(1,0) = get(1,0,3);
        covariance_(2,0) = get(2,0,3);

        covariance_(0,1) = get(0,1,3);
        covariance_(1,1) = get(1,1,3);
        covariance_(2,1) = get(2,1,3);

        covariance_(0,2) = get(0,2,3);
        covariance_(1,2) = get(1,2,3);
        covariance_(2,2) = get(2,2,3);
    }

    void doApply(sample_set_t &sample_set)
    {
        const typename sample_set_t::sample_vector_t &p_t_1 = sample_set.getSamples();
        const std::size_t size = p_t_1.size();
        assert(size != 0);

        /// build the cumulative sums
        SampleDensity2D::ConstPtr density = std::dynamic_pointer_cast<SampleDensity2D const>(sample_set.getDensity());
        if(!density)
            throw std::runtime_error("[KLD2D] : Can only use 'SampleDensity2D' for adaptive sample size estimation!");


        std::cerr << "+" << std::setprecision(20)  << sample_set.getAverageWeight() * static_cast<double>(sample_set.getSampleSize()) << std::endl;
        std::cerr << "|" << std::setprecision(20)  << sample_set.getWeightDistribution().getVariance() << std::endl;
        std::cerr << "|" << std::setprecision(20)  << sample_set.getWeightDistribution().getStandardDeviation() << std::endl;
        std::cerr << "|" << std::setprecision(20)  << sample_set.getMinimumWeight() << std::endl;
        std::cerr << "|" << std::setprecision(20)  << sample_set.getMaximumWeight() << std::endl;
        std::cerr << "|" << std::setprecision(20)  << sample_set.getMinimumWeight() / sample_set.getMaximumWeight() << std::endl;
        std::cerr << "--------------------------------------------" << std::endl;

        if(sample_set.getWeightDistribution().getVariance() < (0.5 * 1e-6)) {
            sample_set.updateDensity();
            return;
        }

//        if(sample_set.getWeightDistribution().getVariance() < 1e4) {

//            SampleDensity2D::state_t mean;
//            SampleDensity2D::covariance_t cov;
//            density->mean(mean, cov);
//            normal_pose_sampler_->apply(mean, covariance_, sample_set);
//            return;
//        }

        const std::size_t sample_size_minimum = std::max(sample_set.getMinimumSampleSize(), 2ul);
        const std::size_t sample_size_maximum = sample_set.getMaximumSampleSize();

        auto kld = [this, &density, sample_size_maximum](const std::size_t current_size){
            const std::size_t k = density->histogramSize();
            const double fraction = 2.0 / (9.0 * (k-1));
            const double exponent = 1.0 - fraction + std::sqrt(fraction) * kld_z_;
            const std::size_t n = std::ceil((k - 1) / (2.0 * kld_error_) * exponent * exponent * exponent);
            return current_size > std::min(n, sample_size_maximum);

        };

        sample_set_t::sample_insertion_t i_p_t = sample_set.getInsertion();

        /// prepare ordered sequence of random numbers
        std::vector<double> cumsum(size + 1, 0.0);
        for(std::size_t i = 0 ; i < size ; ++i) {
            cumsum[i+1] = cumsum[i] + p_t_1[i].weight;
        }
        cslibs_math::random::Uniform<1> rng(0.0, 1.0);
        for(std::size_t i = 0 ; i < sample_size_maximum ; ++i) {
            const double u = rng.get();
            for(std::size_t j = 0 ; j < size ; ++j) {
                if(cumsum[j] <= u && u < cumsum[j+1]) {
                    i_p_t.insert(p_t_1[j]);
                    break;
                }
            }
            if(i > sample_size_minimum && kld(i)) {
                break;
            }
        }
    }

    void doApplyRecovery(sample_set_t &sample_set)
    {
        const auto &p_t_1 = sample_set.getSamples();
        auto  i_p_t = sample_set.getInsertion();
        const std::size_t size = p_t_1.size();

        cslibs_math::random::Uniform<1> rng(0.0, 1.0);

        Sample2D sample;
        const std::size_t sample_size_minimum = std::max(sample_set.getMinimumSampleSize(), 2ul);
        const std::size_t sample_size_maximum = sample_set.getMaximumSampleSize();

        SampleDensity2D::ConstPtr density = std::dynamic_pointer_cast<SampleDensity2D const>(sample_set.getDensity());
        if(!density)
            throw std::runtime_error("[KLD2D] : Can only use 'SampleDensity2D' for adaptive sample size estimation!");

        auto kld = [this, &sample_set, &density, sample_size_maximum](const std::size_t current_size){
            const std::size_t k = density->histogramSize();
            const double fraction = 2.0 / (9.0 * (k-1));
            const double exponent = 1.0 - fraction + std::sqrt(fraction) * kld_z_;
            const std::size_t n = std::ceil((k - 1) / (2.0 * kld_error_) * exponent * exponent * exponent);
            return current_size > std::min(n, sample_size_maximum);

        };

        std::vector<double> cumsum(size + 1, 0.0);
        for(std::size_t i = 0 ; i < size ; ++i) {
            cumsum[i+1] = cumsum[i] + p_t_1[i].weight;
        }
        cslibs_math::random::Uniform<1> rng_recovery(0.0, 1.0);
        for(std::size_t i = 0 ; i < sample_size_maximum ; ++i) {
            const double recovery_probability = rng_recovery.get();
            if(recovery_probability < recovery_random_pose_probability_) {
                uniform_pose_sampler_->apply(sample);
                i_p_t.insert(sample);
            } else {
                const double u = rng.get();
                for(std::size_t j = 0 ; j < size ; ++j) {
                    if(cumsum[j] <= u && u < cumsum[j+1]) {
                        i_p_t.insert(p_t_1[j]);
                        break;
                    }
                }
            }

            if(i > sample_size_minimum && kld(i)) {
                break;
            }
        }
    }

};
}

CLASS_LOADER_REGISTER_CLASS(muse_mcl_2d::LocalRegenerationKLD2D, muse_mcl_2d::Resampling2D)


