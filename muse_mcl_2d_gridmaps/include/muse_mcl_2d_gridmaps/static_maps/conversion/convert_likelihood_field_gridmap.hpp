#ifndef CONVERT_LIKELIHOOD_FIELD_GRIDMAP_HPP
#define CONVERT_LIKELIHOOD_FIELD_GRIDMAP_HPP

#include <muse_mcl_2d_gridmaps/static_maps/likelihood_field_gridmap.h>
#include <muse_mcl_2d_gridmaps/static_maps/algorithms/distance_transform.hpp>

#include <tf/tf.h>
#include <nav_msgs/OccupancyGrid.h>

namespace muse_mcl_2d_gridmaps {
namespace static_maps {
namespace conversion {
inline void from(const nav_msgs::OccupancyGrid &src,
                 LikelihoodFieldGridMap::Ptr &dst,
                 const double maximum_distance = 2.0,
                 const double sigma_hit = 0.5,
                 const double threshold = 1.0)
{
    assert(threshold <= 1.0);
    assert(threshold >= 0.0);
    const double exp_factor_hit = (0.5 * 1.0 / (sigma_hit * sigma_hit));

    muse_mcl_2d::math::Pose2D origin(src.info.origin.position.x,
                                     src.info.origin.position.y,
                                     tf::getYaw(src.info.origin.orientation));

    dst.reset(new LikelihoodFieldGridMap(origin,
                                         src.info.resolution,
                                         src.info.height,
                                         src.info.width,
                                         maximum_distance,
                                         sigma_hit,
                                         src.header.frame_id));
    std::vector<int8_t> occ;
    occ.reserve(src.data.size());
    std::transform(src.data.begin(),
                   src.data.end(),
                   std::back_inserter(occ),
                   [](const int8_t p){return p != -1 ? p : 50;});

    /// 1.) calculate the distances
    algorithms::DistanceTransform<int8_t> distance_transform(src.info.resolution,
                                                             maximum_distance,
                                                             static_cast<int8_t>(threshold * 100));
    distance_transform.apply(occ,
                             dst->getWidth(),
                             dst->getData());

    /// 2.) pre-calculation of the hit likelihoods
    std::for_each(dst->getData().begin(),
                  dst->getData().end(),
                  [exp_factor_hit] (const double z) {return std::exp(-z * z * exp_factor_hit);});
}
}
}
}

#endif // CONVERT_LIKELIHOOD_FIELD_GRIDMAP_HPP
