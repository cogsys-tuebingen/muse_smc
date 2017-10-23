#ifndef POINTCLOUD_2D_HPP
#define POINTCLOUD_2D_HPP

#include <cslibs_math_2d/types/pointcloud.hpp>

namespace muse_mcl_2d_mapping {
class Pointcloud2D  : public cslibs_math_2d::Pointcloud2d
{
public:
    using Ptr = std::shared_ptr<Pointcloud2D>;

    Pointcloud2D(const std::string                 &frame,
                 const cslibs_math_2d::Transform2d &origin) :
        origin_(origin),
        min_(std::numeric_limits<double>::max(),
             std::numeric_limits<double>::max()),
        max_(std::numeric_limits<double>::lowest(),
             std::numeric_limits<double>::lowest())
    {
    }

    Pointcloud2D(const std::string                 &frame,
                 const cslibs_math_2d::Transform2d &origin,
                 const std::size_t size) :
        cslibs_math_2d::Pointcloud2d(size),
        origin_(origin),
        min_(std::numeric_limits<double>::max(),
             std::numeric_limits<double>::max()),
        max_(std::numeric_limits<double>::lowest(),
             std::numeric_limits<double>::lowest())
    {
    }

    inline cslibs_math_2d::Transform2d const & getOrigin() const
    {
        return origin_;
    }

private:
    std::string                 frame_id_;
    cslibs_math_2d::Transform2d origin_;
    cslibs_math_2d::Point2d     min_;
    cslibs_math_2d::Point2d     max_;

};
}

#endif // POINTCLOUD_2D_HPP
