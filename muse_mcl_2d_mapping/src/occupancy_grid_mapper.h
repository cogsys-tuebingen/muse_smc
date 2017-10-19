#ifndef OCCUPANCY_GRID_MAPPER_H
#define OCCUPANCY_GRID_MAPPER_H

#include <atomic>
#include <thread>
#include <condition_variable>
#include <mutex>
#include <future>

#include <muse_mcl_2d_mapping/pointcloud_2d.hpp>
#include <muse_smc/utility/synchronized_queue.hpp>

#include <muse_mcl_2d_gridmaps/static_maps/probability_gridmap.h>
#include <muse_mcl_2d_gridmaps/dynamic_maps/probability_gridmap.h>
#include <muse_mcl_2d_gridmaps/utility/inverse_model.hpp>


namespace muse_mcl_2d_mapping {
class OccupancyGridMapper
{
public:
    using Ptr           = std::shared_ptr<OccupancyGridMapper>;
    using lock_t        = std::unique_lock<std::mutex>;
    using dynamic_map_t = muse_mcl_2d_gridmaps::dynamic_maps::ProbabilityGridMap;
    using static_map_t  = muse_mcl_2d_gridmaps::static_maps::ProbabilityGridMap;


    OccupancyGridMapper(const muse_mcl_2d_gridmaps::utility::InverseModel &inverse_model,
                        const double resolution,
                        const double chunk_resolution,
                        const std::string &frame_id);

    virtual ~OccupancyGridMapper();

    void insert(const Pointcloud2D::Ptr &points);
    static_map_t::Ptr get();


protected:
    /// todo - maybe build an base class
    muse_smc::synchronized::queue<Pointcloud2D::Ptr>            q_;

    std::thread                                                 thread_;
    std::condition_variable                                     notify_event_;
    std::mutex                                                  notify_event_mutex_;
    std::atomic_bool                                            stop_;
    std::atomic_bool                                            request_map_;
    std::promise<static_map_t::Ptr>                             promise_map_;

    dynamic_map_t::Ptr                                          map_;
    muse_mcl_2d_gridmaps::utility::InverseModel                 inverse_model_;
    double                                                      resolution_;
    double                                                      chunk_resolution_;
    std::string                                                 frame_id_;

    void loop();
    void process(const Pointcloud2D::Ptr &points);
    void buildMap();

};
}

#endif // OCCUPANCY_GRID_MAPPER_H
