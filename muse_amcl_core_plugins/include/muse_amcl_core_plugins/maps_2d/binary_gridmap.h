
#include "gridmap.hpp"
#include <nav_msgs/OccupancyGrid.h>

namespace muse_amcl {
namespace maps {
class BinaryGridMap : public GridMap<int8_t>
{
public:
    using Ptr = std::shared_ptr<BinaryGridMap>;

    BinaryGridMap(const nav_msgs::OccupancyGrid &occupancy_grid,
                  const double threshold = 1.0);
    BinaryGridMap(const nav_msgs::OccupancyGrid::ConstPtr &occupancy_grid,
                  const double threshold = 1.0);

    double getRange(const math::Point &from, const math::Point &to) const;

    virtual bool validate(const math::Pose &p) const;


private:
    void convert(const nav_msgs::OccupancyGrid &occupancy_grid,
                 const double threshold);

};
}
}
