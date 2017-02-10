#ifndef ODOMETRY_POSE_2D_HPP
#define ODOMETRY_POSE_2D_HPP

#include <muse_amcl/math/pose.hpp>
#include <muse_amcl/data_types/data.hpp>
#include <muse_amcl/math/angle.hpp>

namespace muse_amcl {
class Odometry : public Data {
public:
    using Ptr = std::shared_ptr<Odometry>;

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

    inline void setPoses(const math::Pose &start,
                         const math::Pose &end)
    {
        start_pose_ = start;
        end_pose_ = end;
        delta_.setEigen3D({end.x() - start.x(), end.y() - start.y(),
                           math::angle::difference(end.yaw(), start.yaw())});

        delta_linear_  = delta_.getOrigin().length();
        delta_angular_ = delta_.yaw();
    }

    inline const math::Pose& getStartPose() const
    {
        return start_pose_;
    }

    inline const math::Pose &getEndPose() const
    {
        return end_pose_;
    }

    inline const math::Pose &getDelta() const
    {
        return delta_;
    }

    inline double getDeltaLinear() const
    {
        return delta_linear_;
    }

    inline double getDeltaAngular() const
    {
        return delta_angular_;
    }

private:
    math::Pose start_pose_;
    math::Pose end_pose_;
    math::Pose delta_;
    double     delta_linear_;
    double     delta_angular_;



};
}

#endif // ODOMETRY_POSE_2D_HPP
