#include <muse_mcl_2d_gridmaps/static_maps/binary_gridmap.h>
#include <tf/tf.h>

using namespace muse_mcl_2d;
using namespace muse_mcl_2d_gridmaps;
using namespace static_maps;

BinaryGridMap::BinaryGridMap(const pose_t &origin,
                             const double resolution,
                             const std::size_t height,
                             const std::size_t width,
                             const std::string &frame_id,
                             const state_t default_value) :
    GridMap<int>(origin,
                 resolution,
                 height,
                 width,
                 default_value,
                 frame_id)
{
}

double BinaryGridMap::getRange(const cslibs_math_2d::Point2d &from,
                               cslibs_math_2d::Point2d &to) const
{
    const_line_iterator_t it = getConstLineIterator(from, to);
    while(it.iterate()) {
        if(*it)
            break;
    }
    if(it.invalid())
        return -1.0;

    fromIndex({it.x(), it.y()}, to);
    return  to.distance(from);
}

double BinaryGridMap::getRange2(const cslibs_math_2d::Point2d &from,
                                cslibs_math_2d::Point2d &to) const
{
    const_line_iterator_t it = getConstLineIterator(from, to);
    while(it.iterate()) {
        if(*it)
            break;
    }
    if(it.invalid())
        return -1.0;

    fromIndex({it.x(), it.y()}, to);
    return  to.distance2(from);
}


bool BinaryGridMap::validate(const cslibs_math_2d::Pose2d &p) const
{
    index_t index;
    if(toIndex(p.translation(), index))
        return at(index[0], index[1]) == 0;
    return false;
}
