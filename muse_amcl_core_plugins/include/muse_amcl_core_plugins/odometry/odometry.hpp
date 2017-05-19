#ifndef ODOMETRY_POSE_2D_HPP
#define ODOMETRY_POSE_2D_HPP

#include <muse_amcl/math/pose.hpp>
#include <muse_amcl/data_types/data.hpp>
#include <muse_amcl/math/angle.hpp>

namespace muse_mcl {
class Odometry : public Data {
public:
    using Ptr = std::shared_ptr<Odometry>;
    using ConstPtr = std::shared_ptr<const Odometry>;

    Odometry(const std::string &frame) :
        Data(frame),
        delta_linear_(0.0),
        delta_angular_(0.0)
    {
    }

    Odometry(const std::string &frame,
             const TimeFrame &time_frame) :
        Data(frame, time_frame),
        delta_linear_(0.0),
        delta_angular_(0.0)
    {
    }

    Odometry(const std::string &frame,
             const TimeFrame &time_frame,
             const math::Pose &start,
             const math::Pose &end) :
        Data(frame, time_frame)
    {
        setPoses(start, end);
    }

    inline double getDeltaAngularAbs() const
    {
        return std::atan2(delta_lin_abs_.y(),
                          delta_lin_abs_.x());
    }

    inline const math::Pose& getStartPose() const
    {
        return start_pose_;
    }

    inline const math::Pose &getEndPose() const
    {
        return end_pose_;
    }

    inline const tf::Vector3 &getDelta() const
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

    bool split(const ros::Time &split_time, Odometry::ConstPtr &a, Odometry::ConstPtr &b) const
    {
        if(!time_frame_.within(split_time))
            return false;

        const double ratio = (split_time - time_frame_.start).toSec() /
                              time_frame_.duration().toSec();
        math::Pose split_pose;
        split_pose.getOrigin().setInterpolate3(start_pose_.getOrigin(), end_pose_.getOrigin(), ratio);
        split_pose.setRotation(tf::slerp(start_pose_.getRotation(), end_pose_.getRotation(), ratio));
        a.reset(new Odometry(frame_, TimeFrame(time_frame_.start, split_time), start_pose_, split_pose));
        b.reset(new Odometry(frame_, TimeFrame(split_time, time_frame_.end), split_pose, end_pose_));
        return true;
    }

private:
    math::Pose  start_pose_;
    math::Pose  end_pose_;

    math::Pose  delta_rel_;

    tf::Vector3 delta_lin_abs_;
    double      delta_linear_;
    double      delta_angular_;

    inline void setPoses(const math::Pose &start,
                         const math::Pose &end)
    {
        delta_rel_  = start.inverse() * end;
        start_pose_ = start;
        end_pose_   = end;

        delta_lin_abs_ = end.getOrigin() - start.getOrigin();
        delta_linear_  = delta_rel_.getOrigin().length();
        delta_angular_ = delta_rel_.yaw();
    }
};
}

#endif // ODOMETRY_POSE_2D_HPP
