#ifndef ODOMETRY_POSE_2D_HPP
#define ODOMETRY_POSE_2D_HPP

#include <muse_mcl_2d/math/pose_2d.hpp>

#include <muse_smc/math/angle.hpp>
#include <muse_smc/data/data.hpp>

#include <ostream>

namespace muse_mcl_2d_odometry {
class Odometry : public muse_smc::Data {
public:
    using Ptr      = std::shared_ptr<Odometry>;
    using ConstPtr = std::shared_ptr<const Odometry>;

    Odometry(const std::string &frame) :
        Data(frame),
        delta_linear_(0.0),
        delta_angular_(0.0)
    {
    }

    Odometry(const std::string &frame,
             const muse_smc::TimeFrame &time_frame) :
        Data(frame, time_frame),
        delta_linear_(0.0),
        delta_angular_(0.0)
    {
    }

    Odometry(const std::string &frame,
             const muse_smc::TimeFrame &time_frame,
             const muse_mcl_2d::Pose2D &start,
             const muse_mcl_2d::Pose2D &end) :
        Data(frame, time_frame)
    {
        setPoses(start, end);
    }

    inline double getDeltaAngularAbs() const
    {
        return std::atan2(delta_lin_abs_.y(),
                          delta_lin_abs_.x());
    }

    inline const muse_mcl_2d::Pose2D& getStartPose() const
    {
        return start_pose_;
    }

    inline const muse_mcl_2d::Pose2D& getEndPose() const
    {
        return end_pose_;
    }

    inline const muse_mcl_2d::Vector2D &getDelta() const
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

    bool split(const muse_smc::Time &split_time, Odometry::ConstPtr &a, Odometry::ConstPtr &b) const
    {
        if(!time_frame_.within(split_time))
            return false;



        const double ratio = (split_time - time_frame_.start).seconds() /
                              time_frame_.duration().seconds();
        muse_mcl_2d::Pose2D interpolated = start_pose_.interpolate(end_pose_, ratio);
        a.reset(new Odometry(frame_, muse_smc::TimeFrame(time_frame_.start, split_time), start_pose_, interpolated));
        b.reset(new Odometry(frame_, muse_smc::TimeFrame(split_time, time_frame_.end), interpolated, end_pose_));
        return true;
    }

private:
    muse_mcl_2d::Pose2D     start_pose_;
    muse_mcl_2d::Pose2D     end_pose_;

    muse_mcl_2d::Pose2D     delta_rel_;

    muse_mcl_2d::Vector2D   delta_lin_abs_;
    double                  delta_linear_;
    double                  delta_angular_;

    inline void setPoses(const muse_mcl_2d::Pose2D &start,
                         const muse_mcl_2d::Pose2D &end)
    {
        delta_rel_  = start.inverse() * end;
        start_pose_ = start;
        end_pose_   = end;

        delta_lin_abs_ = end.translation() - end.translation();
        delta_linear_  = delta_rel_.translation().length();
        delta_angular_ = delta_rel_.yaw();
    }
};
}

inline std::ostream & operator << (std::ostream &out, const muse_mcl_2d_odometry::Odometry &odom)
{
    out << "[Odometry]: linear  " << odom.getDeltaLinear() << std::endl;
    out << "            angular " << odom.getDeltaAngular() << std::endl;
    out << "            delta   " << odom.getDelta().x() << " " << odom.getDelta().y() << std::endl;
    return out;
}

#endif // ODOMETRY_POSE_2D_HPP
