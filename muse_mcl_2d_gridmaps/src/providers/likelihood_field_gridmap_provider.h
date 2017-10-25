#ifndef LIKELIHOOD_FIELD_GRIDMAP_PROVIDER_H
#define LIKELIHOOD_FIELD_GRIDMAP_PROVIDER_H

#include <nav_msgs/OccupancyGrid.h>

#include <mutex>
#include <thread>
#include <atomic>
#include <condition_variable>

#include <muse_mcl_2d/map/map_provider_2d.hpp>
#include <muse_mcl_2d_gridmaps/static_maps/likelihood_field_gridmap.h>


namespace muse_mcl_2d_gridmaps {
class LikelihoodFieldGridmapProvider : public muse_mcl_2d::MapProvider2D
{
public:
    LikelihoodFieldGridmapProvider();

    state_space_t::ConstPtr getStateSpace() const override;
    void setup(ros::NodeHandle &nh) override;

protected:
    ros::Subscriber                             source_;
    std::string                                 topic_;
    double                                      binarization_threshold_;
    double                                      maximum_distance_;
    double                                      z_hit_;
    double                                      sigma_hit_;
    bool                                        blocking_;


    mutable std::mutex                          map_mutex_;
    mutable std::condition_variable             map_loaded_;
    static_maps::LikelihoodFieldGridmap::Ptr    map_;
    std::atomic_bool                            loading_;
    std::thread                                 worker_;

    void callback(const nav_msgs::OccupancyGridConstPtr &msg);


};
}

#endif // LIKELIHOOD_FIELD_GRIDMAP_PROVIDER_H
