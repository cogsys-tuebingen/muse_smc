#ifndef LIKELIHOOD_FIELD_GRIDMAP_SERVICE_PROVIDER_H
#define LIKELIHOOD_FIELD_GRIDMAP_SERVICE_PROVIDER_H


#include <nav_msgs/GetMap.h>

#include <mutex>
#include <thread>
#include <atomic>
#include <condition_variable>

#include <muse_mcl_2d/map/map_provider_2d.hpp>
#include <muse_mcl_2d_gridmaps/static_maps/likelihood_field_gridmap.h>


namespace muse_mcl_2d_gridmaps {
class LikelihoodFieldGridmapServiceProvider : public muse_mcl_2d::MapProvider2D
{
public:
    LikelihoodFieldGridmapServiceProvider();

    state_space_t::ConstPtr getStateSpace() const override;
    void setup(ros::NodeHandle &nh) override;

protected:
    mutable ros::ServiceClient                          source_;
    std::string                                         service_name_;
    double                                              binarization_threshold_;
    double                                              maximum_distance_;
    double                                              sigma_hit_;
    bool                                                blocking_;

    mutable std::mutex                                  map_mutex_;
    mutable std::condition_variable                     map_loaded_;
    mutable static_maps::LikelihoodFieldGridmap::Ptr    map_;
    mutable std::atomic_bool                            loading_;
    mutable std::thread                                 worker_;


};
}

#endif // LIKELIHOOD_FIELD_GRIDMAP_SERVICE_PROVIDER_H
