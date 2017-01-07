#ifndef RESAMPLING_HPP
#define RESAMPLING_HPP

#include <memory>
#include <vector>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <ros/node_handle.h>

#include <muse_amcl/particle_filter/particle_set.hpp>

namespace muse_amcl {
class Resampling {
public:
    typedef std::shared_ptr<Resampling> Ptr;

    Resampling()
    {
    }

    virtual ~Resampling()
    {
    }

    inline const static std::string Type()
    {
        return "muse_amcl::PoseGeneration";
    }

    inline std::string getName() const
    {
        return name_;
    }

    void setup(const std::string &name,
               ros::NodeHandle &nh_private)
    {
        name_ = name;
    }

    virtual void apply(ParticleSet &particle_set) = 0;

protected:
    std::string name_;

    virtual void doSetup(ros::NodeHandle &nh_private) = 0;

    std::string param (const std::string &name)
    {
        return name_ + "/" + name;
    }
};
}

#endif /* RESAMPLING_HPP */
