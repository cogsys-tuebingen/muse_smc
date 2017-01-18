#ifndef KLD2D_H
#define KLD2D_H

#include <muse_amcl/particle_filter/resampling.hpp>
#include <muse_amcl/particle_filter/clustering.hpp>

namespace muse_amcl {
class KLD2D : public Resampling
{
public:
    KLD2D() = default;


    virtual void apply(ParticleSet &particle_set) override;

protected:

    virtual void doSetup(ros::NodeHandle &nh_private) override;

    clustering::KDTreeBufferedPtr  kdtree_;
    clustering::ArrayPtr           array_;
    clustering::Indexation::Size   array_size_;
    clustering::IndexationPtr      index_;

};
}

#endif // KLD2D_H
