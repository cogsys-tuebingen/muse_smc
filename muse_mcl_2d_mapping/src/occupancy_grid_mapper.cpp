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


void OccupancyGridMapper::insert(const Measurement &measurement)
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

    if(static_map_) {
        map = static_map_stamped_t(static_map_t::Ptr(new static_map_t(*static_map_)),
                                   static_map_time_);
    }
}


void OccupancyGridMapper::get(static_map_stamped_t &map,
                              allocated_chunks_t &chunks)
{
    request_map_ = true;
    lock_t static_map_lock(static_map_mutex_);
    notify_event_.notify_one();
    notify_static_map_.wait(static_map_lock);

    if(static_map_) {
        map = static_map_stamped_t(static_map_t::Ptr(new static_map_t(*static_map_)),
                                   static_map_time_);
        chunks = allocated_chunks_;
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
        static_map_.reset(new static_map_t(origin,
                                           dynamic_map_->getResolution(),
                                           dynamic_map_->getHeight(),
                                           dynamic_map_->getWidth()));
        allocated_chunks_.clear();
        static_map_time_ = latest_time_;

        const std::size_t chunk_step = dynamic_map_->getChunkSize();
        const dynamic_map_t::index_t min_chunk_index = dynamic_map_->getMinChunkIndex();
        const dynamic_map_t::index_t max_chunk_index = dynamic_map_->getMaxChunkIndex();
        for(int i = min_chunk_index[1] ; i <= max_chunk_index[1] ; ++i) {
            for(int j = min_chunk_index[0] ; j <= max_chunk_index[0] ; ++j) {
                const dynamic_map_t::chunk_t *chunk = dynamic_map_->getChunk({{j,i}});
                if(chunk != nullptr) {
                    const std::size_t cx = static_cast<std::size_t>((j - min_chunk_index[0]) * static_cast<int>(chunk_step));
                    const std::size_t cy = static_cast<std::size_t>((i - min_chunk_index[1]) * static_cast<int>(chunk_step));

                    cslibs_math_2d::Point2d ll(cx * resolution_, cy * resolution_);
                    cslibs_math_2d::Point2d ru = ll + cslibs_math_2d::Point2d(chunk_step * resolution_);
                    allocated_chunks_.emplace_back(cslibs_math_2d::Box2d(origin * ll, origin * ru));

                    for(std::size_t k = 0 ; k < chunk_step ; ++k) {
                        for(std::size_t l = 0 ; l < chunk_step ; ++l) {
                            static_map_->at(cx + l, cy + k) = chunk->at(l,k);
                        }
                    }
                }
            }
        }

        cslibs_gridmaps::static_maps::conversion::LogOdds::from(static_map_, static_map_);
    }
    request_map_ = false;
    notify_static_map_.notify_one();
}

void OccupancyGridMapper::process(const Measurement &m)
{
    if(!dynamic_map_) {
        dynamic_map_pose_ = m.origin;
        dynamic_map_.reset(new dynamic_map_t(cslibs_math_2d::Transform2d::identity(),
                                             resolution_,
                                             chunk_resolution_,
                                             inverse_model_.getLogOddsPrior()));
        latest_time_ = m.stamp;
    }

    auto discretize = [this](const double x)
    {
        return static_cast<int>(std::floor(x / resolution_));
    };

    if(m.stamp > latest_time_) {
        latest_time_ = m.stamp;
    }


    const dynamic_map_t::index_t      start_index = {{discretize(m.origin.translation()(0)),
                                                      discretize(m.origin.translation()(1))}};

    const double resolution2 = (resolution_ * resolution_ * 0.25);
    for(auto it = m.points->begin() ; it != m.points->end() ; ++it) {
        if(it->isNormal()) {
            const cslibs_math_2d::Point2d end_point = m.origin * *it;
            const dynamic_map_t::index_t  end_index = {{discretize(end_point(0)),
                                                        discretize(end_point(1))}};
            auto b = dynamic_map_->getLineIterator(start_index,
                                                   end_index);

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
