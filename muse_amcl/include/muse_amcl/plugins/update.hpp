#pragma once

#include <memory>
#include <vector>
#include <tf/tf.h>
#include <ros/node_handle.h>

#include <muse_amcl/pf/particle_set.hpp>

namespace muse_amcl {
class Update {
public:
    typedef std::shared_ptr<Update> Ptr;

    Update()
    {
    }

    inline const static std::string Type()
    {
        return "muse_amcl::Update";
    }

    void setup(const std::string &name,
               ros::NodeHandle   &nh)
    {
        name_ = name;
        loadParameters(nh);
    }

    virtual double apply(ParticleSet::WeightIterator set) = 0;

protected:
    std::string name_;

    virtual void loadParameters(ros::NodeHandle &nh) = 0;
};
}
