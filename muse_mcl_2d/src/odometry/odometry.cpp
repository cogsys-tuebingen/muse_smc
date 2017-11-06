#include <muse_mcl_2d/odometry/odometry_2d.h>

namespace muse_mcl_2d_odometry {

Odometry2D::Odometry2D(const std::string &frame) :
    Data(frame),
    start_pose_(cslibs_math_2d::Transform2d::identity()),
    end_pose_(cslibs_math_2d::Transform2d::identity()),
    delta_rel_(cslibs_math_2d::Transform2d::identity()),
    delta_linear_(0.0),
    delta_angular_(0.0)
{
}

Odometry2D::Odometry2D(const std::string &frame,
                       const time_frame_t &time_frame) :
    Data(frame, time_frame),
    start_pose_(cslibs_math_2d::Transform2d::identity()),
    end_pose_(cslibs_math_2d::Transform2d::identity()),
    delta_rel_(cslibs_math_2d::Transform2d::identity()),
    delta_linear_(0.0),
    delta_angular_(0.0)
{
}

Odometry2D::Odometry2D(const std::string &frame,
                       const time_frame_t &time_frame,
                       const cslibs_math_2d::Pose2d &start,
                       const cslibs_math_2d::Pose2d &end) :
    Data(frame, time_frame),
    start_pose_(start),
    end_pose_(start),
    delta_rel_(cslibs_math_2d::Transform2d::identity())
{
    delta_rel_      = start.inverse() * end;
    start_pose_     = start;
    end_pose_       = end;

    delta_lin_abs_  = end.translation() - start.translation();
    delta_linear_   = delta_rel_.translation().length();
    delta_angular_  = delta_rel_.yaw();
}

double Odometry2D::getDeltaAngularAbs() const
{
    return std::atan2(delta_lin_abs_.y(),
                      delta_lin_abs_.x());
}

const cslibs_math_2d::Pose2d& Odometry2D::getStartPose() const
{
    return start_pose_;
}

const cslibs_math_2d::Pose2d& Odometry2D::getEndPose() const
{
    return end_pose_;
}

const cslibs_math_2d::Vector2d& Odometry2D::getDelta() const
{
    return delta_lin_abs_;
}

double Odometry2D::getDeltaLinear() const
{
    return delta_linear_;
}

double Odometry2D::getDeltaAngular() const
{
    return delta_angular_;
}

bool Odometry2D::split(const time_t &split_time, Odometry2D::ConstPtr &a, Odometry2D::ConstPtr &b) const
{
    if(!time_frame_.within(split_time))
        return false;

    const double ratio = (split_time - time_frame_.start).seconds() /
            time_frame_.duration().seconds();

    cslibs_math_2d::Pose2d split_pose = start_pose_.interpolate(end_pose_, ratio);
    a.reset(new Odometry2D(frame_, time_frame_t(time_frame_.start, split_time), start_pose_, split_pose));
    b.reset(new Odometry2D(frame_, time_frame_t(split_time, time_frame_.end), split_pose, end_pose_));
    return true;
}
}
