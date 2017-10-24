#include "occupancy_grid_mapper.h"

#include <muse_mcl_2d_gridmaps/static_maps/conversion/convert_probability_gridmap.hpp>

using namespace muse_mcl_2d_mapping;

/// have a look http://en.cppreference.com/w/cpp/thread/condition_variable

OccupancyGridMapper::OccupancyGridMapper(const muse_mcl_2d_gridmaps::utility::InverseModel &inverse_model,
                                         const double                                       resolution,
                                         const double                                       chunk_resolution,
                                         const std::string                                 &frame_id) :
    stop_(false),
    request_map_(false),
    inverse_model_(inverse_model),
    resolution_(resolution),
    chunk_resolution_(chunk_resolution),
    frame_id_(frame_id)

{
    thread_ = std::thread([this](){loop();});
    thread_.detach();

}

OccupancyGridMapper::~OccupancyGridMapper()
{
    stop_ = true;
    notify_event_.notify_one();
    if(thread_.joinable())
        thread_.join();
}


void OccupancyGridMapper::insert(const Measurement &measurement)
{
    q_.emplace(measurement);
    notify_event_.notify_one();
}


OccupancyGridMapper::static_map_t::Ptr OccupancyGridMapper::get()
{
    ROS_INFO_STREAM("Requested a map!");
    request_map_ = true;
    lock_t notify_map_lock(notify_map_mutex_);
    notify_event_.notify_one();
    notify_map_.wait(notify_map_lock);
    ROS_INFO_STREAM("Got something!");
    return static_map_;
}


void OccupancyGridMapper::loop()
{
    lock_t notify_event_mutex_lock(notify_event_mutex_);
    while(!stop_) {
        notify_event_.wait(notify_event_mutex_lock);
        ROS_ERROR_STREAM("Got notification!");
        while(q_.hasElements()) {
            if(stop_)
                break;
            if(request_map_) {
                ROS_ERROR_STREAM("Process request #1!");
                buildMap();
                request_map_ = false;
            }
            auto m = q_.pop();
            process(m);
        }
        if(request_map_) {
            ROS_ERROR_STREAM("Process request #2!");
            buildMap();
            request_map_ = false;
        }
    }
}

void OccupancyGridMapper::process(const Measurement &m)
{
    ROS_ERROR_STREAM("Processing");
    if(!map_) {
        const cslibs_math_2d::Pose2d &p = m.origin;
        map_.reset(new dynamic_map_t(p,
                                     resolution_,
                                     chunk_resolution_,
                                     frame_id_,
                                     inverse_model_.getLogOddsPrior()));
    }

    const double resolution_2 = resolution_ * 0.5;
    for(auto it = m.points->begin() ; it != m.points->end() ; ++it) {
        if(it->isNormal()) {
            auto b_ptr = map_->getLineIterator(m.origin.translation(),
                                               *it);
            auto &b = *b_ptr;

            while(!b.done()) {
                *b = b.length2() > resolution_2 ? inverse_model_.updateFree(*b) : inverse_model_.updateOccupied(*b);
                ++b;
            }
            *b = inverse_model_.updateOccupied(*b);
        }
    }
}

void OccupancyGridMapper::buildMap()
{
    ROS_INFO_STREAM("Trying building a map!");
    if(map_) {
        ROS_INFO_STREAM("Building a map!");
        static_map_.reset(new static_map_t(map_->getOrigin(),
                                           map_->getResolution(),
                                           map_->getHeight(),
                                           map_->getWidth(),
                                           map_->getFrame()));

        const int chunk_step = map_->getChunkSize();
        const dynamic_map_t::index_t min_chunk_index = map_->getMinChunkIndex();
        const dynamic_map_t::index_t max_chunk_index = map_->getMaxChunkIndex();
        for(int i = min_chunk_index[1] ; i < max_chunk_index[1] ; ++i) {
            for(int j = min_chunk_index[0] ; j < max_chunk_index[0] ; ++j) {
                const dynamic_map_t::chunk_t *chunk = map_->getChunk({j,i});
                if(chunk != nullptr) {
                    const int cx = (j - min_chunk_index[0]) * chunk_step;
                    const int cy = (i - min_chunk_index[1]) * chunk_step;

                    for(int k = 0 ; k < chunk_step ; ++k) {
                        for(int l = 0 ; l < chunk_step ; ++l) {
                            static_map_->at(cx + l, cy + k) = chunk->at(l,k);
                        }
                    }
                }
            }
        }

        muse_mcl_2d_gridmaps::static_maps::conversion::LogOdds::from(static_map_, static_map_);
        ROS_INFO_STREAM("Finished building a map.");
    } else {
        ROS_INFO_STREAM("Map was empty.");
    }
    notify_map_.notify_one();
}
