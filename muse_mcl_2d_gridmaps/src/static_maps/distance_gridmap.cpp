#include <muse_mcl_2d_gridmaps/static_maps/distance_gridmap.h>
#include <muse_mcl_2d_gridmaps/static_maps/algorithms/distance_transform.hpp>

#include <tf/tf.h>


using namespace muse_mcl_2d_gridmaps;
using namespace muse_mcl_2d;
using namespace static_maps;

DistanceGridMap::DistanceGridMap(const pose_t &origin,
                                 const double resolution,
                                 const double maximum_distance,
                                 const std::size_t height,
                                 const std::size_t width,
                                 const std::string &frame_id,
                                 const double default_value) :
    GridMap<double>(origin,
                    resolution,
                    height,
                    width,
                    default_value,
                    frame_id),
    maximum_distance_(maximum_distance)
{
}

double DistanceGridMap::getMaximumDistance() const
{
    return maximum_distance_;
}

double DistanceGridMap::at(const muse_mcl_math_2d::Point2D &point) const
{
    index_t i;
    toIndex(point, i);
    if(invalid(i))
        return maximum_distance_;
    return GridMap<double>::at(i[0], i[1]);
}
