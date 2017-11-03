#include "occupancy_grid_mapper.h"

#include <muse_mcl_2d_gridmaps/static_maps/conversion/convert_probability_gridmap.hpp>
#include <cslibs_math/common/array.hpp>

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

void OccupancyGridMapper::get(static_map_t::Ptr &map)
{
    request_map_ = true;
    lock_t static_map_lock(static_map_mutex_);
    notify_event_.notify_one();
    notify_static_map_.wait(static_map_lock);

    if(static_map_) {
        map.reset(new OccupancyGridMapper::static_map_t(*static_map_));
    }
}


void OccupancyGridMapper::get(muse_mcl_2d_gridmaps::static_maps::ProbabilityGridmap::Ptr &map,
                              allocated_chunks_t                                         &chunks)
{
    request_map_ = true;
    lock_t static_map_lock(static_map_mutex_);
    notify_event_.notify_one();
    notify_static_map_.wait(static_map_lock);

    if(static_map_) {
        map.reset(new OccupancyGridMapper::static_map_t(*static_map_));
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
    if(request_map_ && map_) {
        static_map_.reset(new static_map_t(map_->getInitialOrigin(),
                                           map_->getResolution(),
                                           map_->getHeight(),
                                           map_->getWidth(),
                                           map_->getFrame()));
        allocated_chunks_.clear();

        const int chunk_step = map_->getChunkSize();
        const dynamic_map_t::index_t min_chunk_index = map_->getMinChunkIndex();
        const dynamic_map_t::index_t max_chunk_index = map_->getMaxChunkIndex();
        for(int i = min_chunk_index[1] ; i <= max_chunk_index[1] ; ++i) {
            for(int j = min_chunk_index[0] ; j <= max_chunk_index[0] ; ++j) {
                const dynamic_map_t::chunk_t *chunk = map_->getChunk({j,i});
                if(chunk != nullptr) {
                    const int cx = (j - min_chunk_index[0]) * chunk_step;
                    const int cy = (i - min_chunk_index[1]) * chunk_step;

                    allocated_chunks_.emplace_back(cslibs_math_2d::Box2d(cx * resolution_,
                                                                         cy * resolution_,
                                                                         (cx + chunk_step) * resolution_,
                                                                         (cy + chunk_step) * resolution_));

                    for(int k = 0 ; k < chunk_step ; ++k) {
                        for(int l = 0 ; l < chunk_step ; ++l) {
                            static_map_->at(cx + l, cy + k) = chunk->at(l,k);
                        }
                    }
                }
            }
        }

        muse_mcl_2d_gridmaps::static_maps::conversion::LogOdds::from(static_map_, static_map_);
    }
    request_map_ = false;
    notify_static_map_.notify_one();
}

void OccupancyGridMapper::process(const Measurement &m)
{
    if(!map_) {
        const cslibs_math_2d::Pose2d &p = m.origin;
        map_.reset(new dynamic_map_t(cslibs_math_2d::Transform2d::identity(),
                                     resolution_,
                                     chunk_resolution_,
                                     frame_id_,
                                     inverse_model_.getLogOddsPrior()));
    }

    auto discretize = [this](const double x)
    {
        return static_cast<int>(std::floor(x / resolution_));
    };

    const cslibs_math_2d::Transform2d o_T_m = map_->getInitialOrigin();
    const cslibs_math_2d::Transform2d m_T_o = o_T_m.inverse();
    const cslibs_math_2d::Transform2d m_T_l = m_T_o * m.origin;
    const dynamic_map_t::index_t      start_index = {{discretize(m.origin.translation().x()),
                                                      discretize(m.origin.translation().y())}};

    const double resolution2 = (resolution_ * resolution_ * 0.25);
    for(auto it = m.points->begin() ; it != m.points->end() ; ++it) {
        if(it->isNormal()) {
            const cslibs_math_2d::Point2d end_point = m.origin * *it;
            const dynamic_map_t::index_t  end_index = {{discretize(end_point.x()),
                                                        discretize(end_point.y())}};

            auto b = map_->getLineIterator(start_index,
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
