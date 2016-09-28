#pragma once

#include <tf/tf.h>

namespace muse_amcl {
namespace particle_filter{

class SampleWeight
{
public:
    SampleWeight() :
        weight(-1.0)
    {
    }

    SampleWeight(const double weight) :
        weight(weight)
    {
    }

    inline void setWeight(const double w)
    {
        weight = w;
    }

    inline double getWeight() const
    {
        return weight;
    }
    inline void getWeight(double &w)
    {
        w = weight;
    }

protected:
   double   weight;
};

class SamplePose
{
public:
    SamplePose()
    {
        pose.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
        pose.setRotation(tf::createQuaternionFromYaw(0.0));
    }

    SamplePose(const tf::Pose &pose) :
        pose(pose)
    {
    }

    inline void setPose(const tf::Pose &p)
    {
        pose = p;
    }

    inline void updatePose(const tf::Transform &t)
    {
        pose = t * pose;
    }

    inline tf::Pose getPose() const
    {
        return pose;
    }
    inline void getPose(tf::Pose &p) const
    {
        p = pose;
    }

protected:
    tf::Pose pose;
};

class Sample : public SampleWeight, public SamplePose
{
public:
    Sample() :
        SampleWeight(),
        SamplePose()
    {
    }

    Sample(const tf::Pose pose,
           const double weight) :
        SampleWeight(weight),
        SamplePose(pose)
    {
    }
};
}
}
