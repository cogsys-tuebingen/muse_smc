#ifndef POINTCLOUD_2D_HPP
#define POINTCLOUD_2D_HPP

#include <muse_smc/data/data.hpp>
#include <cslibs_math_2d/point_2d.hpp>
#include <cslibs_math_2d/pose_2d.hpp>

namespace muse_mcl_2d_mapping {
class Pointcloud2D  : public muse_smc::Data
{
public:
    struct Point2D {
        const muse_mcl_math_2d::Point2D point;
        const bool                       valid;

        inline Point2D() :
            point(std::numeric_limits<double>::infinity(),
                  std::numeric_limits<double>::infinity()),
            valid(false)
        {
        }

        inline Point2D(const muse_mcl_math_2d::Point2D &point,
                       const bool valid = true) :
                point(point),
                valid(valid)
        {
        }

        inline Point2D(const Point2D &other) :
            point(other.point),
            valid(other.valid)
        {
        }

        inline Point2D(Point2D &&other) :
            point(std::move(other.point)),
            valid(other.valid)
        {
        }
    } __attribute__ ((aligned (32)));

    using Ptr = std::shared_ptr<Pointcloud2D>;
    using time_frame_t = muse_smc::TimeFrame;
    using points_t = std::vector<Point2D>;
    using const_iterator_t = points_t::const_iterator;

    Pointcloud2D(const std::string &frame,
                 const time_frame_t &time_frame,
                 const muse_mcl_math_2d::Pose2D &origin) :
        muse_smc::Data(frame, time_frame),
        origin_(origin),
        min_(std::numeric_limits<double>::max(),
             std::numeric_limits<double>::max()),
        max_(std::numeric_limits<double>::lowest(),
             std::numeric_limits<double>::lowest())
    {
    }

    Pointcloud2D(const std::string &frame,
                 const time_frame_t &time_frame,
                 const muse_mcl_math_2d::Pose2D &origin,
                 const std::size_t size) :
        muse_smc::Data(frame, time_frame),
        points_(size),
        origin_(origin),
        min_(std::numeric_limits<double>::max(),
             std::numeric_limits<double>::max()),
        max_(std::numeric_limits<double>::lowest(),
             std::numeric_limits<double>::lowest())
    {
    }


    inline void insert(const muse_mcl_math_2d::Point2D &pt)
    {
        min_ = min_.min(pt);
        max_ = max_.max(pt);
        points_.emplace_back(pt);
    }

    inline void insertInvalid()
    {
        points_.emplace_back(Point2D());
    }

    inline const_iterator_t begin() const
    {
        return points_.begin();
    }

    inline const_iterator_t end() const
    {
        return points_.end();
    }

    inline std::vector<Point2D> const & getPoints() const
    {
        return points_;
    }

    inline muse_mcl_math_2d::Pose2D const & getOrigin() const
    {
        return origin_;
    }

private:
    std::vector<Point2D>       points_;
    muse_mcl_math_2d::Pose2D  origin_;
    muse_mcl_math_2d::Point2D min_;
    muse_mcl_math_2d::Point2D max_;

};
}

#endif // POINTCLOUD_2D_HPP
