#include "kld_2d.h"

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(muse_amcl::KLD2D, muse_amcl::Resampling)


using namespace muse_amcl;

void KLD2D::doApply(ParticleSet &particle_set)
{
    const ParticleSet::Particles &p_t_1 = particle_set.getSamples();
    const double w_max = particle_set.getSampleWeightMaximum();
    ParticleSet::Insertion  i_p_t = particle_set.getInsertion();
    const std::size_t size = p_t_1.size();

    math::random::Uniform<1> rng(0.0, 1.0);
    double beta = 0.0;
    std::size_t index = (std::size_t(rng.get() * size)) % size;

    const std::size_t sample_size_minimum = std::max(particle_set.getSampleSizeMinimum(), 2ul);
    const std::size_t sample_size_maximum = particle_set.getSampleSizeMaximum();

    auto kld = [this, &particle_set, sample_size_maximum](const std::size_t current_size){
        const std::size_t k = particle_set.getCurrentHistogramSize();
        const double fraction = 2.0 / (9.0 * (k-1));
        const double exponent = 1.0 - fraction + std::sqrt(fraction) * kld_z_;
        const std::size_t n = std::ceil((k - 1) / (2.0 * kld_error_) * exponent * exponent * exponent);
        return current_size > std::min(n, sample_size_maximum);

    };

    for(std::size_t i = 1 ; i <= sample_size_maximum ; ++i) {
        beta += 2 * w_max * rng.get();
        while (beta > p_t_1[index].weight_) {
            beta -= p_t_1[index].weight_;
            index = (index + 1) % size;
        }
        i_p_t.insert(p_t_1[index]);

        if(i > sample_size_minimum && kld(i)) {
            break;
        }
    }
}

void KLD2D::doApplyRecovery(ParticleSet &particle_set)
{
    const ParticleSet::Particles &p_t_1 = particle_set.getSamples();
    const double w_max = particle_set.getSampleWeightMaximum();
    ParticleSet::Insertion  i_p_t = particle_set.getInsertion();
    const std::size_t size = p_t_1.size();

    math::random::Uniform<1> rng(0.0, 1.0);
    double beta = 0.0;
    std::size_t index = (std::size_t(rng.get() * size)) % size;

    Particle particle;
    const std::size_t sample_size_minimum = std::max(particle_set.getSampleSizeMinimum(), 2ul);
    const std::size_t sample_size_maximum = particle_set.getSampleSizeMaximum();

    auto kld = [this, &particle_set, sample_size_maximum](const std::size_t current_size){
        const std::size_t k = particle_set.getCurrentHistogramSize();
        const double fraction = 2.0 / (9.0 * (k-1));
        const double exponent = 1.0 - fraction + std::sqrt(fraction) * kld_z_;
        const std::size_t n = std::ceil((k - 1) / (2.0 * kld_error_) * exponent * exponent * exponent);
        return current_size > std::min(n, sample_size_maximum);

    };

    math::random::Uniform<1> rng_recovery(0.0, 1.0);
    for(std::size_t i = 1 ; i <= sample_size_maximum ; ++i) {
        beta += 2 * w_max * rng.get();
        while (beta > p_t_1[index].weight_) {
            beta -= p_t_1[index].weight_;
            index = (index + 1) % size;
        }

        const double recovery_probability = rng_recovery.get();
        if(recovery_probability < recovery_random_pose_probability_) {
            uniform_pose_sampler_->apply(particle);
            i_p_t.insert(particle);
        } else {
            i_p_t.insert(p_t_1[index]);
        }

        if(i > sample_size_minimum && kld(i)) {
            break;
        }
    }
}

void KLD2D::doSetup(ros::NodeHandle &nh_private)
{
    kld_error_ = nh_private.param(parameter("kld_error"), 0.01);
    kld_z_     = nh_private.param(parameter("kld_z"), 0.99);
    //    Indexation::Resolution resolution;    /// the resolution for array and kdtree
    //    resolution[0] = nh_private.param(parameter("resolution_linear"), 0.1);
    //    resolution[1] = resolution[0];
    //    resolution[2] = nh_private.param(parameter("resolution_radial"), M_PI / 18.0);

    //    index_.reset(new Indexation(resolution));

    //    if(nh_private.param(parameter("use_array"), false)) {
    //        /// allocate array for convereged resampling
    //        std::array<double, 3> span;
    //        span[0] = nh_private.param(parameter("array_size"), 5.0);
    //        span[1] = span[0];
    //        span[2] = 2 * M_PI;
    //        array_.reset(new Array);
    //        array_size_ = index_->size(span);
    //        array_->set<cis::option::tags::array_size>(array_size_[0], array_size_[1], array_size_[2]);
    //    }
}
