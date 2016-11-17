#pragma once

#include <memory>
#include <vector>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <ros/node_handle.h>

#include <muse_amcl/data_types/data.hpp>
#include <muse_amcl/data_types/map.hpp>
#include <muse_amcl/particle_filter/particle_set.hpp>

namespace muse_amcl {
class Update {
public:
    typedef std::shared_ptr<Update> Ptr;

    Update()
    {
    }

    virtual ~Update()
    {
    }

    inline const static std::string Type()
    {
        return "muse_amcl::Update";
    }

    inline std::string name() const
    {
        return name_;
    }

    void setup(const std::string &name,
               ros::NodeHandle   &nh_private)
    {
        name_ = name;
        doSetup(nh_private);
    }

    virtual double apply(const Data::ConstPtr &data,
                         const Map::ConstPtr &map,
                         ParticleSet::WeightIterator set) = 0;

protected:
    std::string name_;

    virtual void doSetup(ros::NodeHandle &nh_private) = 0;

    std::string param(const std::string &name)
    {
        return name_ + "/" + name;
    }

};
}
