#ifndef ODOMETRY_POSE_2D_HPP
#define ODOMETRY_POSE_2D_HPP

#include <muse_amcl/math/pose.hpp>
#include <muse_amcl/data_types/data.hpp>

namespace muse_amcl {
class Odometry : public Data {
public:
    using Ptr = std::shared_ptr<Odometry>;

    Odometry(const std::string &frame) :
        Data(frame)
    {
    }

    Odometry(const std::string &frame,
             const TimeFrame &time_frame) :
        Data(frame, time_frame)
    {
    }

    inline void setPoses(const math::Pose &start,
                         const math::Pose &end)
    {
        start_pose_ = start;
        end_pose_ = end;
        delta_ = start * end.tf().inverse();
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


private:
    math::Pose start_pose_;
    math::Pose end_pose_;
    math::Pose delta_;



};
}

#endif // ODOMETRY_POSE_2D_HPP
