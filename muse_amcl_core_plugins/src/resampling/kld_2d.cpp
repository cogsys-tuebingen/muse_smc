#include "kld_2d.h"

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(muse_amcl::KLD2D, muse_amcl::Resampling)


using namespace muse_amcl;

void KLD2D::doApply(ParticleSet &particle_set)
{
//    /**
//     * @brief particles
//     *  - resample low variance / any other suitable scheme
//     *  - put into tree
//     *  - cluster
//     *  - resize
//     *  - look after cluster statistics
//     *  - may be put tree and or array into particle set?
//     *  - abstact to be clustering handler ?
//     */

    const ParticleSet::Particles &p_t_1 = particle_set.getSamples();
    const double weight_avergy = particle_set.getWeightAverage();
    ParticleSet::Insertion i_p_t = particle_set.getInsertion();




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
