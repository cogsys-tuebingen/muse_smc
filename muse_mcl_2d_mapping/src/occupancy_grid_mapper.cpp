#include "occupancy_grid_mapper.h"

#include <muse_mcl_2d_gridmaps/static_maps/conversion/convert_probability_gridmap.hpp>

using namespace muse_mcl_2d_mapping;

OccupancyGridMapper::OccupancyGridMapper(const muse_mcl_2d_gridmaps::utility::InverseModel &inverse_model,
                                         const double resolution,
                                         const double chunk_resolution,
                                         const std::string &frame_id) :
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


void OccupancyGridMapper::insert(const Pointcloud2D::Ptr &points)
{
    q_.emplace(points);
}


OccupancyGridMapper::static_map_t::Ptr OccupancyGridMapper::get()
{
    request_map_ = true;
    notify_event_.notify_one();

    std::future<static_map_t::Ptr> f = promise_map_.get_future();
    f.wait();
    return f.get();
}


void OccupancyGridMapper::loop()
{
    lock_t notify_event_mutex_lock(notify_event_mutex_);
    while(!stop_) {
        notify_event_.wait(notify_event_mutex_lock);
        while(q_.hasElements()) {
            if(stop_)
                break;
            if(request_map_)
                buildMap();

            auto e = q_.pop();
            process(e);
        }
        if(request_map_)
            buildMap();
    }
}

void OccupancyGridMapper::process(const Pointcloud2D::Ptr &points)
{
    if(!map_) {
        const muse_mcl_2d::math::Pose2D &p = points->getOrigin();
        map_.reset(new dynamic_map_t(p,
                                     resolution_,
                                     chunk_resolution_,
                                     frame_id_,
                                     inverse_model_.getLogOddsPrior()));
    }

    const double resolution_2 = resolution_ * 0.5;
    for(auto it = points->begin() ; it != points->end() ; ++it) {
        if(it->valid) {
            auto b_ptr = map_->getLineIterator(points->getOrigin().translation(),
                                               it->point);
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
    static_map_t::Ptr built_map(new static_map_t(map_->getOrigin(),
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
                auto l = chunk->lock();
                const int cx = (j - min_chunk_index[0]) * chunk_step;
                const int cy = (i - min_chunk_index[1]) * chunk_step;

                for(int k = 0 ; k < chunk_step ; ++k) {
                    for(int l = 0 ; l < chunk_step ; ++l) {
                        built_map->at(cx + l, cy + k) = chunk->at(l,k);
                    }
                }
            }
        }
    }

    muse_mcl_2d_gridmaps::static_maps::conversion::LogOdds::from(built_map, built_map);
    promise_map_.set_value(built_map);
}
