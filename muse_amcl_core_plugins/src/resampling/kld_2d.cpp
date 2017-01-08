#include "kld_2d.h"

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(muse_amcl::KLD2D, muse_amcl::Resampling)

#include "kld_2d_tree.hpp"

using namespace muse_amcl;

void KLD2D::apply(ParticleSet &particle_set)
{

}

void KLD2D::doSetup(ros::NodeHandle &nh_private)
{
    resolution_linear_ = nh_private.param(parameter("resolution_linear"), 0.05);
    resolution_radial_ = nh_private.param(parameter("resolution_radial"), M_PI / 18.0);

    if(nh_private.param(parameter("voxel_grid"), false)) {
        voxel_grid_size_ = nh_private.param("voxel_grid_size", 5.0);
        /// allocate voxel grid
    }
}
