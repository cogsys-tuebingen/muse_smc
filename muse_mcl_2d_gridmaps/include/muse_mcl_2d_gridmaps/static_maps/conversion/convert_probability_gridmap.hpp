#ifndef CONVERT_PROBABILITY_GRIDMAP_HPP
#define CONVERT_PROBABILITY_GRIDMAP_HPP

#include <muse_mcl_2d_gridmaps/static_maps/probability_gridmap.h>

#include <cslibs_math/common/log_odds.hpp>

#include <nav_msgs/OccupancyGrid.h>
#include <tf/tf.h>

namespace muse_mcl_2d_gridmaps {
namespace static_maps {
namespace conversion {
inline void from(const nav_msgs::OccupancyGrid &src,
                 ProbabilityGridmap::Ptr &dst)
{
    cslibs_math_2d::Pose2d origin(src.info.origin.position.x,
                                     src.info.origin.position.y,
                                     tf::getYaw(src.info.origin.orientation));

    dst.reset(new ProbabilityGridmap(origin,
                                     src.info.resolution,
                                     src.info.height,
                                     src.info.width,
                                     src.header.frame_id));
    std::transform(src.data.begin(), src.data.end(),
                   dst->getData().begin(),
                   [](const int8_t p){return p != -1 ? 0.01 * p : 0.5;});
}


inline void from(const nav_msgs::OccupancyGrid::ConstPtr &src,
                 ProbabilityGridmap::Ptr &dst)
{
   from(*src, dst);
}

inline void from(const ProbabilityGridmap::Ptr &src,
                 nav_msgs::OccupancyGrid::Ptr &dst)
{
    dst.reset(new nav_msgs::OccupancyGrid);
    dst->header.stamp            = ros::Time(src->getStamp().seconds());
    dst->header.frame_id         = src->getFrame();
    dst->info.resolution         = src->getResolution();
    dst->info.height             = src->getHeight();
    dst->info.width              = src->getWidth();
    dst->info.origin.position.x  = src->getOrigin().tx();
    dst->info.origin.position.y  = src->getOrigin().ty();
    dst->info.origin.orientation = tf::createQuaternionMsgFromYaw(src->getOrigin().yaw());
    dst->info.map_load_time = dst->header.stamp;
    dst->data.resize(src->getData().size());
    std::transform(src->getData().begin(), src->getData().end(),
                   dst->data.begin(),
                   [](const double p){return p == 0.5 ? -1 : static_cast<int8_t>(p * 100.0);});
}

struct LogOdds {
    static inline void to(ProbabilityGridmap::Ptr &src,
                          ProbabilityGridmap::Ptr &dst)
    {
        if(src != dst) {
            dst.reset(new ProbabilityGridmap(*src));
        }
        std::for_each(dst->getData().begin(),
                      dst->getData().end(),
                      [](double &p){p =  cslibs_math::common::LogOdds::to(p);});

    }
    static inline void from(ProbabilityGridmap::Ptr &src,
                            ProbabilityGridmap::Ptr &dst)
    {
        if(src != dst) {
            dst.reset(new ProbabilityGridmap(*src));
        }
        std::for_each(dst->getData().begin(),
                      dst->getData().end(),
                      [](double &l){l = cslibs_math::common::LogOdds::from(l);});

    }
};
}
}
}

#endif // CONVERT_PROBABILITY_GRIDMAP_HPP
