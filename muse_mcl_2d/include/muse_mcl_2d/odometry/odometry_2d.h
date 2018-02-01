#ifndef ODOMETRY_2D_HPP
#define ODOMETRY_2D_HPP

#include <muse_smc/data/data.hpp>

#include <cslibs_math_2d/linear/pose.hpp>
#include <cslibs_math_2d/linear/transform.hpp>

namespace muse_mcl_2d_odometry {
class Odometry2D : public muse_smc::Data {
public:
    using Ptr          = std::shared_ptr<Odometry2D>;
    using ConstPtr     = std::shared_ptr<const Odometry2D>;
    using time_frame_t = cslibs_time::TimeFrame;
    using time_t       = cslibs_time::Time;

    Odometry2D(const std::string &frame);

    Odometry2D(const std::string &frame,
               const time_frame_t &time_frame);

    Odometry2D(const std::string &frame,
               const time_frame_t &time_frame,
               const cslibs_math_2d::Pose2d &start,
               const cslibs_math_2d::Pose2d &end);

    double  getDeltaAngularAbs() const;
    const   cslibs_math_2d::Pose2d&   getStartPose() const;
    const   cslibs_math_2d::Pose2d&   getEndPose() const;
    const   cslibs_math_2d::Vector2d& getDelta() const;
    double  getDeltaLinear() const;
    double  getDeltaAngular() const;
    bool    split(const time_t &split_time, Odometry2D::ConstPtr &a, Odometry2D::ConstPtr &b) const;

private:
    cslibs_math_2d::Pose2d    start_pose_;
    cslibs_math_2d::Pose2d    end_pose_;
    cslibs_math_2d::Pose2d    delta_rel_;
    cslibs_math_2d::Vector2d  delta_lin_abs_;
    double                    delta_linear_;
    double                    delta_angular_;
}__attribute__ ((aligned (256)));
}

inline std::ostream & operator << (std::ostream &out, const muse_mcl_2d_odometry::Odometry2D &odom)
{
    out << "[Odometry]: linear  " << odom.getDeltaLinear()  << "\n";
    out << "            angular " << odom.getDeltaAngular() << "\n";
    out << "            delta   " << odom.getDelta()(0) << " " << odom.getDelta()(1);
    return out;
}


#endif // ODOMETRY_2D_HPP
