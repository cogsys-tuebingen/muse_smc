#pragma once

#include <atomic>
#include <thread>
#include <condition_variable>
#include <mutex>

#include <cslibs_utility/synchronized/synchronized_queue.hpp>

#include "measurement_3d.hpp"

#include <cslibs_gridmaps/static_maps/algorithms/normalize.hpp>
#include <cslibs_gridmaps/static_maps/probability_gridmap.h>
#include <cslibs_gridmaps/utility/delegate.hpp>
#include <cslibs_ndt_3d/dynamic_maps/gridmap.hpp>
#include <cslibs_time/stamped.hpp>
#include <cslibs_math_3d/linear/pointcloud.hpp>
#include <cslibs_math_3d/linear/box.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace muse_mcl_3d_mapping {
class NDTGridMapper3d
{
public:
    using Ptr                   = std::shared_ptr<NDTGridMapper3d>;
    using lock_t                = std::unique_lock<std::mutex>;
    using dynamic_map_t         = cslibs_ndt_3d::dynamic_maps::Gridmap;
    using static_map_t          = pcl::PointCloud<pcl::PointXYZI>;//cslibs_gridmaps::static_maps::ProbabilityGridmap;
    using static_map_stamped_t  = cslibs_time::Stamped<static_map_t::Ptr>;
    using callback_t            = delegate<void(const static_map_stamped_t &)>;

public:
    NDTGridMapper3d(
            const double        resolution,
            const double        sampling_resolution,
            const std::string & frame_id);

    virtual ~NDTGridMapper3d();

    void insert(
            const Measurement3d & measurement);

    void get(
            static_map_stamped_t & map);
    void requestMap();

    void setCallback(
            const callback_t & cb);

protected:
    cslibs_utility::synchronized::queue<Measurement3d> q_;

    std::thread                                  thread_;
    std::condition_variable                      notify_event_;
    std::mutex                                   notify_event_mutex_;
    std::atomic_bool                             stop_;
    std::atomic_bool                             request_map_;
    std::condition_variable                      notify_static_map_;
    std::mutex                                   static_map_mutex_;

    static_map_stamped_t                         static_map_;

    callback_t                                   callback_;

    cslibs_time::Time                            latest_time_;
    dynamic_map_t::Ptr                           dynamic_map_;
    double                                       resolution_;
    double                                       sampling_resolution_;
    std::string                                  frame_id_;

    void loop();

    void mapRequest();

    void process(
            const Measurement3d & points);
};
}
