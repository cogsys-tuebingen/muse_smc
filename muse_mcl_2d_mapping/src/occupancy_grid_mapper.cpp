#include "occupancy_grid_mapper.h"

#include <cslibs_gridmaps/static_maps/conversion/convert_probability_gridmap.hpp>
#include <cslibs_math/common/array.hpp>

namespace muse_mcl_2d_mapping {
OccupancyGridMapper::OccupancyGridMapper(const cslibs_gridmaps::utility::InverseModel &inverse_model,
                                         const double                                  resolution,
                                         const double                                  chunk_resolution,
                                         const std::string                            &frame_id) :
    stop_(false),
    request_map_(false),
    callback_([](const static_map_t::Ptr &){}),
    inverse_model_(inverse_model),
    resolution_(resolution),
    chunk_resolution_(chunk_resolution),
    frame_id_(frame_id)

{
    thread_ = std::thread([this](){loop();});

}

OccupancyGridMapper::~OccupancyGridMapper()
{
    stop_ = true;
    notify_event_.notify_one();
    if(thread_.joinable())
        thread_.join();
}


void OccupancyGridMapper::insert(const Measurement2d &measurement)
{
    q_.emplace(measurement);
    notify_event_.notify_one();
}

void OccupancyGridMapper::get(static_map_stamped_t &map)
{
    request_map_ = true;
    lock_t static_map_lock(static_map_mutex_);
    notify_event_.notify_one();
    notify_static_map_.wait(static_map_lock);
    map = static_map_;
}

void OccupancyGridMapper::requestMap()
{
    request_map_ = true;
}

void OccupancyGridMapper::setCallback(const callback_t &cb)
{
    if(!request_map_) {
        callback_ = cb;
    }
}

void OccupancyGridMapper::loop()
{
    lock_t notify_event_mutex_lock(notify_event_mutex_);
    while(!stop_) {
        notify_event_.wait(notify_event_mutex_lock);
        while(q_.hasElements()) {
            if(stop_)
                break;

            mapRequest();

            auto m = q_.pop();
            process(m);
        }
        mapRequest();
    }
}

void OccupancyGridMapper::mapRequest()
{
    if(request_map_ && dynamic_map_) {
        cslibs_math_2d::Transform2d origin = dynamic_map_->getOrigin();
        static_map_.data().reset(new static_map_t(origin,
                                                  dynamic_map_->getResolution(),
                                                  dynamic_map_->getHeight(),
                                                  dynamic_map_->getWidth()));

        static_map_.stamp() = latest_time_;

        const std::size_t chunk_step = dynamic_map_->getChunkSize();
        const dynamic_map_t::index_t min_chunk_index = dynamic_map_->getMinChunkIndex();
        const dynamic_map_t::index_t max_chunk_index = dynamic_map_->getMaxChunkIndex();
        for(int i = min_chunk_index[1] ; i <= max_chunk_index[1] ; ++i) {
            for(int j = min_chunk_index[0] ; j <= max_chunk_index[0] ; ++j) {
                dynamic_map_t::chunk_t *chunk = dynamic_map_->getChunk({{j,i}});
                if(chunk != nullptr) {
                    const std::size_t cx = static_cast<std::size_t>((j - min_chunk_index[0]) * static_cast<int>(chunk_step));
                    const std::size_t cy = static_cast<std::size_t>((i - min_chunk_index[1]) * static_cast<int>(chunk_step));
                    for(std::size_t k = 0 ; k < chunk_step ; ++k) {
                        for(std::size_t l = 0 ; l < chunk_step ; ++l) {
                            static_map_.data()->at(cx + l, cy + k) = chunk->at(l,k);
                        }
                    }
                }
            }
        }


        cslibs_gridmaps::static_maps::conversion::LogOdds::from(static_map_, static_map_);
        callback_(static_map_);
    }
    request_map_ = false;
    notify_static_map_.notify_one();
}

void OccupancyGridMapper::process(const Measurement2d &m)
{
    if(!dynamic_map_) {
        dynamic_map_.reset(new dynamic_map_t(cslibs_math_2d::Transform2d::identity(),
                                             resolution_,
                                             chunk_resolution_,
                                             inverse_model_.getLogOddsPrior()));
        latest_time_ = m.stamp;
    }

    if(m.stamp > latest_time_) {
        latest_time_ = m.stamp;
    }

    const double resolution2 = (resolution_ * resolution_ * 0.25);
    for(const auto &p : *(m.points)) {
        if(p.isNormal()) {
            const cslibs_math_2d::Point2d end_point = m.origin * p;
            auto b = dynamic_map_->getLineIterator(m.origin.translation(),
                                                   end_point);

            while(!b.done()) {
                double l = b.length2() > resolution2 ? inverse_model_.updateFree(*b) : inverse_model_.updateOccupied(*b);
                *b = l;
                ++b;
            }
            *b = inverse_model_.updateOccupied(*b);
        }
    }
}
}
