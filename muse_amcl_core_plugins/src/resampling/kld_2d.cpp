#include "kld_2d.h"

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(muse_amcl::KLD2D, muse_amcl::Resampling)


using namespace muse_amcl;
using Indexation = clustering::Indexation;
using KDTree     = clustering::KDTreeBuffered;
using Array      = clustering::Array;
using Clustering = clustering::Clustering;
using Particles  = std::vector<Particle>;

void KLD2D::apply(ParticleSet &particle_set)
{
    /// analyze extend
    if(!kdtree_) {
        kdtree_.reset(new KDTree);
        kdtree_->set<cis::option::tags::node_allocator_chunk_size>(2 * particle_set.getMaximumSize() + 1);
    }

    /**
     * @brief particles
     *  - resample low variance / any other suitable scheme
     *  - put into tree
     *  - cluster
     *  - resize
     *  - look after cluster statistics
     *  - may be put tree and or array into particle set?
     *  - abstact to be clustering handler ?
     */


    Particles  particles = particle_set.getParticles();
    Clustering clusters;
    if(array_) {
        Eigen::Vector3d min = Eigen::Vector3d::Constant(std::numeric_limits<double>::max());
        Eigen::Vector3d max = Eigen::Vector3d::Constant(std::numeric_limits<double>::min());

        bool can_use_array = false;
        for(const Particle &p : particles) {
            Eigen::Vector3d pose = p.pose_.eigen3D();
            for(std::size_t i = 0 ; i < 3 ; ++i) {
                if(min(i) > pose(i))
                    min(i) = pose(i);
                if(max(i) < pose(i))
                    max(i) = pose(i);
            }
        }

        Indexation::Size  size = index_->size({min(0), min(1), min(2)},
                                              {max(0), max(1), max(2)});

        can_use_array = size[0] <= array_size_[0] &&
                            size[1] <= array_size_[1] &&
                                size[2] <= array_size_[2];

        if(can_use_array) {
            Indexation::Index min_index = index_->create({min(0), min(1), min(2)});
            array_->clear();
            array_->set<cis::option::tags::array_offset>(min_index[0], min_index[1], min_index[2]);
            clustering::create(*index_, particles, *array_);
            clustering::cluster(*array_, clusters);
        } else {
            kdtree_->clear();
            clustering::create(*index_, particles, *kdtree_);
            clustering::cluster(*kdtree_, clusters);
        }
    } else {
        /// directly used kdtree
        kdtree_->clear();
        clustering::create(*index_, particle_set.getParticles(), *kdtree_);
        clustering::cluster(*kdtree_, clusters);
    }
}

void KLD2D::doSetup(ros::NodeHandle &nh_private)
{
    Indexation::Resolution resolution;    /// the resolution for array and kdtree
    resolution[0] = nh_private.param(parameter("resolution_linear"), 0.1);
    resolution[1] = resolution[0];
    resolution[2] = nh_private.param(parameter("resolution_radial"), M_PI / 18.0);

    index_.reset(new Indexation(resolution));

    if(nh_private.param(parameter("use_array"), false)) {
        /// allocate array for convereged resampling
        std::array<double, 3> span;
        span[0] = nh_private.param(parameter("array_size"), 5.0);
        span[1] = span[0];
        span[2] = 2 * M_PI;
        array_.reset(new Array);
        array_size_ = index_->size(span);
        array_->set<cis::option::tags::array_size>(array_size_[0], array_size_[1], array_size_[2]);
    }


}
