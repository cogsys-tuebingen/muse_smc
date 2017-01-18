#include "residual.h"

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(muse_amcl::Residual, muse_amcl::Resampling)

#include "impl/residual.hpp"

using namespace muse_amcl;

void Residual::apply(ParticleSet &particle_set)
{
    /// initalize particle new particle set
    ParticleSet::Particles &p_t_1 = particle_set.getParticles();
    ParticleSet::Particles  p_t;
    resampling::impl::Residual::apply(p_t_1, p_t, particle_set.getMaximumSize());
    /// assign new content
    assert(p_t.size() == p_t_1.size());
    std::swap(p_t, p_t_1);
}

void Residual::doSetup(ros::NodeHandle &nh_private)
{
}
