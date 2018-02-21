#ifndef MUSE_MCL_2D_NDT_OCCUPANCY_GRIDMAP_3D_PROVIDER_H
#define MUSE_MCL_2D_NDT_OCCUPANCY_GRIDMAP_3D_PROVIDER_H

#include <mutex>
#include <thread>
#include <atomic>
#include <condition_variable>

#include <muse_mcl_2d/map/map_provider_2d.hpp>
#include <muse_mcl_2d_ndt/maps/occupancy_gridmap_3d.h>

#include <sensor_msgs/PointCloud2.h>

namespace muse_mcl_2d_ndt {
class NDTOccupancyGridmap3dProvider : public muse_mcl_2d::MapProvider2D
{
public:
    NDTOccupancyGridmap3dProvider();

    state_space_t::ConstPtr getStateSpace() const override;
    void setup(ros::NodeHandle &nh) override;

protected:
    std::string                     path_;
    std::string                     frame_id_;
    bool                            blocking_;

    mutable std::mutex              map_mutex_;
    OccupancyGridmap3d::Ptr         map_;
    mutable std::mutex              map_load_mutex_;
    std::thread                     worker_;
    mutable std::condition_variable notify_;

    mutable ros::Publisher          pub_;
    double                          threshold_;
    cslibs_gridmaps::utility::InverseModel::Ptr inverse_model_;
    mutable sensor_msgs::PointCloud2::Ptr msg_;

    void loadMap();
    void publishMap() const;
};
}

#endif // MUSE_MCL_2D_NDT_OCCUPANCY_GRIDMAP_3D_PROVIDER_H
