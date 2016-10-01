#pragma once

#include <tf/tf.h>

namespace muse {
struct Particle {
    Particle() :
       pose(tf::createQuaternionFromRPY(0,0,0),
            tf::Vector3(0,0,0)),
       weight(0.0)
    {
    }

    Particle(const tf::Quaternion &_q,
             const tf::Vector3    &_v,
             const double         &_w) :
        pose(_q, _v),
        weight(_w)
    {
    }

    Particle(const tf::Pose &_p,
             const double _w)  :
        pose(_p),
        weight(_w)
    {
    }

    tf::Pose pose;
    double weight;

};
}
