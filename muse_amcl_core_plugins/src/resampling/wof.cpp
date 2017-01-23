#include "wof.h"

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(muse_amcl::WheelOfFortune, muse_amcl::Resampling)

#include "impl/wof.hpp"

using namespace muse_amcl;

void WheelOfFortune::apply(ParticleSet &p_t_1)
{

    ParticleSet::Particles &p_t_1 = p_t_1.getParticles();
    ParticleSet::Particles  p_t;
    resampling::impl::WOF::apply(p_t_1, p_t, p_t_1.getMaximumWeight(), p_t_1.getMaximumSize());
    /// assign new content
    assert(p_t.size() == p_t_1.size());
    std::swap(p_t, p_t_1);
}

void WheelOfFortune::doSetup(ros::NodeHandle &nh_private)
{
}
