#include <muse_smc/math/random.hpp>

#include <muse_mcl_2d/resampling/resampling_2d.hpp>
#include <muse_mcl_2d/samples/sample_density_2d.hpp>


namespace muse_mcl_2d {
class KLD2D : public Resampling2D
{
public:

protected:
    double  kld_error_;
    double  kld_z_;

    virtual void doSetup(ros::NodeHandle &nh) override
    {
        auto param_name = [this](const std::string &name){return name_ + "/" + name;};
        kld_error_ = nh.param(param_name("kld_error"), 0.01);
        kld_z_     = nh.param(param_name("kld_z"), 0.99);
    }

    void doApply(sample_set_t &sample_set)
    {
        const typename sample_set_t::sample_vector_t &p_t_1 = sample_set.getSamples();
        const std::size_t size = p_t_1.size();
        if(size == 0) {
            return;
        }

        const double w_max = sample_set.getMaximumWeight();
        typename sample_set_t::sample_insertion_t i_p_t = sample_set.getInsertion();

        muse_smc::math::random::Uniform<1> rng(0.0, 1.0);
        double beta = 0.0;
        std::size_t index = (std::size_t(rng.get() * size)) % size;

        SampleDensity2D::ConstPtr density = std::dynamic_pointer_cast<SampleDensity2D const>(sample_set.getDensity());
        if(!density)
            throw std::runtime_error("[KLD2D] : Can only use 'SampleDensity2D' for adaptive sample size estimation!");

        const std::size_t sample_size_minimum = std::max(sample_set.getMinimumSampleSize(), 2ul);
        const std::size_t sample_size_maximum = sample_set.getMaximumSampleSize();

        auto kld = [this, &sample_set, &density, sample_size_maximum](const std::size_t current_size){
            const std::size_t k = density->histogramSize();
            const double fraction = 2.0 / (9.0 * (k-1));
            const double exponent = 1.0 - fraction + std::sqrt(fraction) * kld_z_;
            const std::size_t n = std::ceil((k - 1) / (2.0 * kld_error_) * exponent * exponent * exponent);
            return current_size > std::min(n, sample_size_maximum);

        };

        for(std::size_t i = 1 ; i <= sample_size_maximum ; ++i) {
            beta += 2 * w_max * rng.get();
            while (beta > p_t_1[index].weight) {
                beta -= p_t_1[index].weight;
                index = (index + 1) % size;
            }
            i_p_t.insert(p_t_1[index]);

            if(i > sample_size_minimum && kld(i)) {
                break;
            }
        }
    }

    void doApplyRecovery(sample_set_t &sample_set)
    {
        const auto &p_t_1 = sample_set.getSamples();
        const double w_max = sample_set.getMaximumWeight();
        auto  i_p_t = sample_set.getInsertion();
        const std::size_t size = p_t_1.size();

        muse_smc::math::random::Uniform<1> rng(0.0, 1.0);
        double beta = 0.0;
        std::size_t index = (std::size_t(rng.get() * size)) % size;

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

        muse_smc::math::random::Uniform<1> rng_recovery(0.0, 1.0);
        for(std::size_t i = 1 ; i <= sample_size_maximum ; ++i) {
            beta += 2 * w_max * rng.get();
            while (beta > p_t_1[index].weight) {
                beta -= p_t_1[index].weight;
                index = (index + 1) % size;
            }

            const double recovery_probability = rng_recovery.get();
            if(recovery_probability < recovery_random_pose_probability_) {
                uniform_pose_sampler_->apply(sample);
                i_p_t.insert(sample);
            } else {
                i_p_t.insert(p_t_1[index]);
            }

            if(i > sample_size_minimum && kld(i)) {
                break;
            }
        }
    }

};
}

CLASS_LOADER_REGISTER_CLASS(muse_mcl_2d::KLD2D, muse_mcl_2d::Resampling2D)


