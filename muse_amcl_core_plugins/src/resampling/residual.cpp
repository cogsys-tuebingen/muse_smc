#include "residual.h"

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(muse_amcl::Residual, muse_amcl::Resampling)

#include <muse_amcl/math/random.hpp>

using namespace muse_amcl;

void Residual::doSetup(ros::NodeHandle &nh_private)
{
}

void Residual::doApply(ParticleSet &particle_set)
{
    const ParticleSet::Particles &p_t_1 = particle_set.getSamples();
    const std::size_t size = p_t_1.size();
    if(size == 0) {
        std::cerr << "[Residual]: Cannot resample empty set!" << std::endl;
        return;
    }

    Insertion i_p_t = particle_set.getInsertion();

    std::vector<double> u(size);
    std::vector<double> w_residual(size);
    double              n_w_residual = 0.0;
    std::size_t         i_p_t_size = 0;
    {
        math::random::Uniform<1> rng(0.0, 1.0);
        double u_static = rng.get();
        for(std::size_t i = 0 ; i < size ; ++i) {
            const auto &sample = p_t_1[i];
            u[i] = (i + u_static) / size;
            std::size_t copies = std::floor(sample.weight_ * size);

            w_residual[i] = size * sample.weight_ - copies;
            n_w_residual += w_residual[i];

            for(std::size_t i = 0 ; i < copies && i_p_t_size < size ;
                ++i ,++i_p_t_size) {
                i_p_t.insert(sample);
            }
        }
    }
    {
        auto u_it = u.begin();
        auto p_t_1_it = p_t_1.begin();
        auto w_it = w_residual.begin();

        double cumsum_last = 0.0;
        double cumsum = 0.0;
        auto in_range = [&cumsum, &cumsum_last] (double u)
        {
            return u >= cumsum_last && u < cumsum;
        };

        for(std::size_t i = i_p_t_size ; i < size ; ++i) {
            while(!in_range(*u_it)) {
                ++p_t_1_it;
                ++w_it;
                cumsum_last = cumsum;
                cumsum += *w_it / n_w_residual;
            }
            i_p_t.insert(*p_t_1_it);
            ++u_it;
        }
    }
}

void Residual::doApplyRecovery(ParticleSet &particle_set)
{
    uniform_pose_sampler_->update(particle_set.getFrame());

    const ParticleSet::Particles &p_t_1 = particle_set.getSamples();
    Insertion i_p_t = particle_set.getInsertion();

    math::random::Uniform<1> rng_recovery(0.0, 1.0);

    const std::size_t size = p_t_1.size();
    std::vector<double> u(size);
    std::vector<double> w_residual(size);
    double              n_w_residual = 0.0;
    std::size_t         i_p_t_size = 0;
    {
        math::random::Uniform<1> rng(0.0, 1.0);
        double u_static = rng.get();
        for(std::size_t i = 0 ; i < size ; ++i) {
            const auto &sample = p_t_1[i];
            u[i] = (i + u_static) / size;
            std::size_t copies = std::floor(sample.weight_ * size);

            w_residual[i] = size * sample.weight_ - copies;
            n_w_residual += w_residual[i];

            Particle particle;
            for(std::size_t i = 0 ; i < copies && i_p_t_size < size ;
                ++i ,++i_p_t_size) {
                const double recovery_probability = rng_recovery.get();
                if(recovery_probability < recovery_random_pose_probability_) {
                    uniform_pose_sampler_->apply(particle);
                    particle.weight_ = recovery_probability;
                    i_p_t.insert(particle);
                } else {
                    i_p_t.insert(sample);
                }
            }
        }
    }
    {
        auto u_it = u.begin();
        auto p_t_1_it = p_t_1.begin();
        auto w_it = w_residual.begin();

        double cumsum_last = 0.0;
        double cumsum = 0.0;
        auto in_range = [&cumsum, &cumsum_last] (double u)
        {
            return u >= cumsum_last && u < cumsum;
        };

        Particle particle;
        for(std::size_t i = i_p_t_size ; i < size ; ++i) {
            while(!in_range(*u_it)) {
                ++p_t_1_it;
                ++w_it;
                cumsum_last = cumsum;
                cumsum += *w_it / n_w_residual;
            }

            const double recovery_probability = rng_recovery.get();
            if(recovery_probability < recovery_random_pose_probability_) {
                uniform_pose_sampler_->apply(particle);
                particle.weight_ = recovery_probability;
                i_p_t.insert(particle);
            } else {
                i_p_t.insert(*p_t_1_it);
            }
            ++u_it;
        }
    }
}
