#ifndef MUSE_MCL_2D_NDT_GRIDMAP_PROVIDER_H
#define MUSE_MCL_2D_NDT_GRIDMAP_PROVIDER_H

#include <mutex>
#include <thread>
#include <atomic>
#include <condition_variable>

#include <muse_mcl_2d/map/map_provider_2d.hpp>
#include <muse_mcl_2d_ndt/maps/gridmap.h>

namespace muse_mcl_2d_ndt {
class NDTGridmapProvider : public muse_mcl_2d::MapProvider2D
{
public:
    NDTGridmapProvider();

    state_space_t::ConstPtr getStateSpace() const override;
    void setup(ros::NodeHandle &nh) override;

protected:
    std::string                     path_;
    std::string                     frame_id_;
    bool                            blocking_;

    mutable std::mutex              map_mutex_;
    mutable std::condition_variable map_loaded_;
    Gridmap::Ptr                    map_;
    std::atomic_bool                loading_;
    std::thread                     worker_;

    mutable ros::Publisher          pub_;
    double                          sampling_resolution_;

    void loadMap();
    void publishMap() const;
};
}

#endif // MUSE_MCL_2D_NDT_GRIDMAP_PROVIDER_H
