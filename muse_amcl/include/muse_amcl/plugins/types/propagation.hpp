#pragma once

#include <memory>
#include <vector>
#include <tf/tf.h>
#include <ros/node_handle.h>

#include "../data/data.hpp"
#include <muse_amcl/pf/particle_set.hpp>

namespace muse_amcl {
class Propagation {
public:
    typedef std::shared_ptr<Propagation> Ptr;

    Propagation()
    {
    }

    virtual ~Propagation()
    {
    }

    inline static const std::string Type()
    {
        return "muse_amcl::Propagation";
    }

    inline std::string name() const
    {
        return name_;
    }

    void setup(const std::string &name,
               ros::NodeHandle   &nh)
    {
        name_ = name;
        loadParameters(nh);
    }

    virtual void apply(Data::ConstPtr &data,
                       ParticleSet::PoseIterator set) = 0;

protected:
    std::string name_;

    virtual void loadParameters(ros::NodeHandle &nh) = 0;

    std::string param(const std::string &name)
    {
        return name_ + "/" + name;
    }

};
}
