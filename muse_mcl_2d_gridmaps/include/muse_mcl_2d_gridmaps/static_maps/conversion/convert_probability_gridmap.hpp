#ifndef CONVERT_PROBABILITY_GRIDMAP_HPP
#define CONVERT_PROBABILITY_GRIDMAP_HPP

#include <muse_mcl_2d_gridmaps/static_maps/probability_gridmap.h>
#include <muse_mcl_2d_gridmaps/utility/log_odds.hpp>

#include <nav_msgs/OccupancyGrid.h>
#include <tf/tf.h>

namespace muse_mcl_2d_gridmaps {
namespace static_maps {
namespace conversion {
inline void from(const nav_msgs::OccupancyGrid &src,
                 ProbabilityGridMap::Ptr &dst)
{
    assert(threshold <= 1.0);
    assert(threshold >= 0.0);

    muse_mcl_2d::math::Pose2D origin(src.info.origin.position.x,
                                     src.info.origin.position.y,
                                     tf::getYaw(src.info.origin.orientation));

    dst.reset(new ProbabilityGridMap(origin,
                                     src.info.resolution,
                                     src.info.height,
                                     src.info.width,
                                     src.header.frame_id));
    std::transform(src.data.begin(), src.data.end(),
                   dst->getData().begin(),
                   [](const int8_t p){return p != -1 ? 0.01 * p : 0.5;});
}


inline void from(const nav_msgs::OccupancyGrid::ConstPtr &src,
                 ProbabilityGridMap::Ptr &dst)
{
   from(*src, dst);
}

inline void from(const ProbabilityGridMap::Ptr &src,
                 nav_msgs::OccupancyGrid::Ptr &dst)
{
    dst.reset(new nav_msgs::OccupancyGrid);
    dst->header.stamp       = ros::Time(src->getStamp().seconds());
    dst->header.frame_id    = src->getFrame();
    dst->info.resolution    = src->getResolution();
    dst->info.height        = src->getHeight();
    dst->info.width         = src->getWidth();
    dst->info.map_load_time = dst->header.stamp;
    dst->data.resize(src->getData().size());
    std::transform(src->getData().begin(), src->getData().end(),
                   dst->data.begin(),
                   [](const double p){return p == 0.5 ? -1 : static_cast<int8_t>(p * 100.0);});
}

struct LogOdds {
    static inline void to(ProbabilityGridMap::Ptr &src,
                          ProbabilityGridMap::Ptr &dst)
    {
        if(src != dst) {
            dst.reset(new ProbabilityGridMap(*src));
        }
        std::for_each(dst->getData().begin(),
                      dst->getData().end(),
                      [](const double p){return utility::LogOdds::to(p);});

    }
    static inline void from(ProbabilityGridMap::Ptr &src,
                            ProbabilityGridMap::Ptr &dst)
    {
        if(src != dst) {
            dst.reset(new ProbabilityGridMap(*src));
        }
        std::for_each(dst->getData().begin(),
                      dst->getData().end(),
                      [](const double l){return utility::LogOdds::from(l);});

    }
};
}
}
}

#endif // CONVERT_PROBABILITY_GRIDMAP_HPP
