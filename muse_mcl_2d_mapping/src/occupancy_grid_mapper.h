#ifndef OCCUPANCY_GRID_MAPPER_H
#define OCCUPANCY_GRID_MAPPER_H

#include <atomic>
#include <thread>
#include <condition_variable>
#include <mutex>

#include <cslibs_utility/synchronized/synchronized_queue.hpp>

#include "measurement_2d.hpp"

#include <cslibs_gridmaps/static_maps/probability_gridmap.h>
#include <cslibs_gridmaps/dynamic_maps/probability_gridmap.h>
#include <cslibs_gridmaps/utility/inverse_model.hpp>
#include <cslibs_gridmaps/utility/delegate.hpp>
#include <cslibs_time/stamped.hpp>
#include <cslibs_math_2d/linear/pointcloud.hpp>
#include <cslibs_math_2d/linear/box.hpp>

namespace muse_mcl_2d_mapping {
class OccupancyGridMapper
{
public:
    using Ptr                           = std::shared_ptr<OccupancyGridMapper>;
    using lock_t                        = std::unique_lock<std::mutex>;
    using dynamic_map_t                 = cslibs_gridmaps::dynamic_maps::ProbabilityGridmap;
    using static_map_t                  = cslibs_gridmaps::static_maps::ProbabilityGridmap;
    using static_map_stamped_t          = cslibs_time::Stamped<static_map_t::Ptr>;
    using model_t                       = cslibs_gridmaps::utility::InverseModel;
    using chunks_t                      = std::vector<cslibs_math_2d::Box2d>;
    using callback_t                    = delegate<void(const static_map_stamped_t &,
                                                        const chunks_t &,
                                                        const chunks_t &,
                                                        const chunks_t &)>;

    OccupancyGridMapper(const cslibs_gridmaps::utility::InverseModel &inverse_model,
                        const double resolution,
                        const double chunk_resolution,
                        const std::string &frame_id);

    virtual ~OccupancyGridMapper();

    void insert(const Measurement2d &measurement);

    void get(static_map_stamped_t &map);
    void get(static_map_stamped_t &map, chunks_t &chunks);

    void requestMap();
    void setCallback (const callback_t &cb);

protected:
    cslibs_utility::synchronized::queue<Measurement2d> q_;

    std::thread                                  thread_;
    std::condition_variable                      notify_event_;
    std::mutex                                   notify_event_mutex_;
    std::atomic_bool                             stop_;
    std::atomic_bool                             request_map_;
    std::condition_variable                      notify_static_map_;
    std::mutex                                   static_map_mutex_;

    static_map_stamped_t                         static_map_;
    chunks_t                                     allocated_chunks_;
    chunks_t                                     touched_chunks_;
    chunks_t                                     untouched_chunks_;

    callback_t                                   callback_;

    cslibs_time::Time                            latest_time_;
    dynamic_map_t::Ptr                           dynamic_map_;
    cslibs_gridmaps::utility::InverseModel       inverse_model_;
    double                                       resolution_;
    double                                       chunk_resolution_;
    std::string                                  frame_id_;

    void loop();
    void mapRequest();
    void process(const Measurement2d &points);

};
}

#endif // OCCUPANCY_GRID_MAPPER_H
