#ifndef MUSE_MCL_2D_NDT_GRIDMAP_3D_PROVIDER_H
#define MUSE_MCL_2D_NDT_GRIDMAP_3D_PROVIDER_H

#include <mutex>
#include <thread>
#include <atomic>
#include <condition_variable>

#include <muse_mcl_2d/map/map_provider_2d.hpp>
#include <muse_mcl_2d_ndt/maps/gridmap_3d.h>

namespace muse_mcl_2d_ndt {
class NDTGridmap3dProvider : public muse_mcl_2d::MapProvider2D
{
public:
    NDTGridmap3dProvider();

    state_space_t::ConstPtr getStateSpace() const override;
    void setup(ros::NodeHandle &nh) override;

protected:
    std::string                     path_;
    std::string                     frame_id_;
    bool                            blocking_;

    mutable std::mutex              map_mutex_;
    mutable std::condition_variable map_loaded_;
    Gridmap3d::Ptr                  map_;
    std::atomic_bool                loading_;
    std::thread                     worker_;

    void loadMap();
};
}

#endif // MUSE_MCL_2D_NDT_GRIDMAP_3D_PROVIDER_H
