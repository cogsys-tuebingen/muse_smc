#ifndef WEIGHTED_QUATERNION_MEAN_HPP
#define WEIGHTED_QUATERNION_MEAN_HPP

#include <tf/tf.h>

namespace muse_scm {
namespace math {
namespace statistic {
class WeightedQuaternionMean {
public:
    WeightedQuaternionMean() :
        W_(0.0)
    {
    }

    WeightedQuaternionMean & operator += (const tf::Quaternion &q, const double w)
    {
        if(W_ == 0.0) {
            mean = q * w;
            W_ = w;
        } else {
            W_1_ = W;
            W_ += w;

            if(quaternionsClose(mean, q)) {
                mean = (mean / W_1_ + q * w) / W_;
            } else {
                mean = (mean / W_1_ + q.inverse() * w) / W_;
            }
        }
    }

private:
    double         W_1_;
    double         W_;
    tf::Quaternion mean;


    //Returns true if the two input quaternions are close to each other. This can
    //be used to check whether or not one of two quaternions which are supposed to
    //be very similar but has its component signs reversed (q has the same rotation as
    //-q)
    inline bool quaternionsClose(const tf::Quaternion &q_a,
                                 const tf::Quaternion &q_b)
    {
        double dot = q_a.dot(q_b);
        return dot < 0.0;
    }

};
}
}
}



#endif // WEIGHTED_QUATERNION_MEAN_HPP
