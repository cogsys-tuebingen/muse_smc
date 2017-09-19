#ifndef ODOMETRY_2D_HPP
#define ODOMETRY_2D_HPP

#include <muse_smc/data/data.hpp>

#include <muse_mcl_2d/math/pose_2d.hpp>
#include <muse_mcl_2d/math/transform_2d.hpp>

namespace muse_mcl_2d {
class Odometry2D : public muse_smc::Data {
public:
    using Ptr = std::shared_ptr<Odometry2D>;
    using ConstPtr = std::shared_ptr<const Odometry2D>;
    using time_frame_t = muse_smc::TimeFrame;
    using time_t = muse_smc::Time;

    Odometry2D(const std::string &frame) :
        Data(frame),
        start_pose_(math::Transform2D::identity()),
        end_pose_(math::Transform2D::identity()),
        delta_rel_(math::Transform2D::identity()),
        delta_linear_(0.0),
        delta_angular_(0.0)
    {
    }

    Odometry2D(const std::string &frame,
               const time_frame_t &time_frame) :
        Data(frame, time_frame),
        start_pose_(math::Transform2D::identity()),
        end_pose_(math::Transform2D::identity()),
        delta_rel_(math::Transform2D::identity()),
        delta_linear_(0.0),
        delta_angular_(0.0)
    {
    }

    Odometry2D(const std::string &frame,
              const time_frame_t &time_frame,
              const math::Pose2D &start,
              const math::Pose2D &end) :
       Data(frame, time_frame),
       start_pose_(start),
       end_pose_(start),
       delta_rel_(math::Transform2D::identity())
    {
        delta_rel_      = start.inverse() * end;
        start_pose_     = start;
        end_pose_       = end;

        delta_lin_abs_  = end.translation() - start.translation();
        delta_linear_   = delta_rel_.translation().length();
        delta_angular_  = delta_rel_.yaw();
    }

    inline double getDeltaAngularAbs() const
    {
        return std::atan2(delta_lin_abs_.y(),
                          delta_lin_abs_.x());
    }

    inline const math::Pose2D& getStartPose() const
    {
        return start_pose_;
    }

    inline const math::Pose2D &getEndPose() const
    {
        return end_pose_;
    }

    inline const math::Vector2D &getDelta() const
    {
        return delta_lin_abs_;
    }

    inline double getDeltaLinear() const
    {
        return delta_linear_;
    }

    inline double getDeltaAngular() const
    {
        return delta_angular_;
    }

    bool split(const time_t &split_time, Odometry2D::ConstPtr &a, Odometry2D::ConstPtr &b) const
    {
        if(!time_frame_.within(split_time))
            return false;

        const double ratio = (split_time - time_frame_.start).seconds() /
                              time_frame_.duration().seconds();

        math::Pose2D split_pose = start_pose_.interpolate(end_pose_, ratio);
        a.reset(new Odometry2D(frame_, time_frame_t(time_frame_.start, split_time), start_pose_, split_pose));
        b.reset(new Odometry2D(frame_, time_frame_t(split_time, time_frame_.end), split_pose, end_pose_));
        return true;
    }

private:
    math::Pose2D    start_pose_;
    math::Pose2D    end_pose_;

    math::Pose2D    delta_rel_;

    math::Vector2D  delta_lin_abs_;
    double          delta_linear_;
    double          delta_angular_;
};
}

inline std::ostream & operator << (std::ostream &out, const muse_mcl_2d::Odometry2D &odom)
{
    out << "[Odometry]: linear  " << odom.getDeltaLinear()  << "\n";
    out << "            angular " << odom.getDeltaAngular() << "\n";
    out << "            delta   " << odom.getDelta().x() << " " << odom.getDelta().y();
    return out;
}


#endif // ODOMETRY_2D_HPP
