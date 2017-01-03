#ifndef PROPAGATION_HPP
#define PROPAGATION_HPP

#include <memory>
#include <vector>
#include <tf/tf.h>
#include <ros/node_handle.h>

#include <muse_amcl/data_types/data.hpp>
#include <muse_amcl/particle_filter/particle_set.hpp>

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
        doSetup(nh);
    }

    virtual void apply(const Data::ConstPtr &data,
                       ParticleSet::PoseIterator set) = 0;

    virtual void predict(const std::time_t until,
                         ParticleSet::PoseIterator set)
    {
    }

    virtual bool canPredict(const std::time_t until)
    {
        return false;
    }

protected:
    std::string name_;

    virtual void doSetup(ros::NodeHandle &nh) = 0;

    std::string param(const std::string &name)
    {
        return name_ + "/" + name;
    }

};
}

#endif /* PROPAGATION_HPP */