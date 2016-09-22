#ifndef GRIDMAP_H
#define GRIDMAP_H

#include <tf/tf.h>

namespace muse {
namespace maps {
class GridMap
{
public:
    typedef std::shared_ptr<GridMap> Ptr;

    inline void index(const double x, const double y,
                      int &idx, int &idy)
    {
        if(origin_phi_ == 0.0) {
            idx = (x - origin_x_) / resolution_;
            idy = (y - origin_y_) / resolution_;
        } else {
            double x_ =  cos_origin_phi_ * x + sin_origin_phi_ * y;
            double y_ = -sin_origin_phi_ * x + cos_origin_phi_ * y;
            idx = (x_ - origin_x_) / resolution_;
            idy = (y_ - origin_y_) / resolution_;
        }
    }

    inline void position(const int idx, const int idy,
                         double &x, double &y)
    {
        if(origin_phi_ == 0.0) {
            x = idx * resolution_ + origin_x_;
            y = idy * resolution_ + origin_y_;
        } else {
            double x_ = cos_origin_phi_ * x - sin_origin_phi_ * y;
            double y_ = sin_origin_phi_ * x + cos_origin_phi_ * y;
            x = x_;
            y = y_;
        }
    }

protected:
    double origin_x_;
    double origin_y_;
    double origin_phi_;
    double cos_origin_phi_;
    double sin_origin_phi_;
    double resolution_;

    GridMap(const double origin_x,
            const double origin_y,
            const double origin_phi,
            const double resolution);

    GridMap(const tf::Pose &origin,
            const double resolution);

    GridMap(const GridMap &other) = delete;

};
}
}
#endif // GRIDMAP_H
