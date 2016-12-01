#include "residual.h"

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(muse_amcl::Residual, muse_amcl::Resampling)

using namespace muse_amcl;

void Residual::apply(ParticleSet &particle_set)
{
    /// initalize particle new particle set
    ParticleSet::Particles &p_old = particle_set.getParticles();
    const std::size_t size        = p_old.size();
    ParticleSet::Particles  p_new(size);

    /// draw copies of each particle
    auto p_new_it = p_new.begin();
    auto p_new_end = p_new.end();
    for(const auto &p : p_old) {
        std::size_t copies = std::floor(p.weight_ * size);
        for(std::size_t i = 0 ; i < copies && p_new_it != p_new_end ; ++i ,++p_new_it) {
            *p_new_it = p;
        }
    }

    /// if particles to draw are left, do it systematically
    std::size_t left = std::distance(p_new_it, p_new_end);
    if(left > 0) {

    }
    std::swap(p_old, p_new);
}

void Residual::doSetup(ros::NodeHandle &nh_private)
{

}
