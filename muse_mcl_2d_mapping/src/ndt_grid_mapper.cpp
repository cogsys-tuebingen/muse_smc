#include "ndt_grid_mapper.h"

namespace muse_mcl_2d_mapping {
NDTGridMapper::NDTGridMapper(const double resolution,
                             const double sampling_resolution,
                             const std::string &frame_id) :
    stop_(false),
    request_map_(false),
    callback_([](const static_map_t::Ptr &, const chunks_t &, const chunks_t &, const chunks_t &){}),
    resolution_(resolution),
    sampling_resolution_(sampling_resolution),
    frame_id_(frame_id)

{
    thread_ = std::thread([this](){loop();});
}

NDTGridMapper::~NDTGridMapper()
{
    stop_ = true;
    notify_event_.notify_one();
    if(thread_.joinable())
        thread_.join();
}


void NDTGridMapper::insert(const Measurement2d &measurement)
{
    q_.emplace(measurement);
    notify_event_.notify_one();
}

void NDTGridMapper::get(static_map_stamped_t &map)
{
    request_map_ = true;
    lock_t static_map_lock(static_map_mutex_);
    notify_event_.notify_one();
    notify_static_map_.wait(static_map_lock);
    map = static_map_;
}


void NDTGridMapper::get(static_map_stamped_t &map,
                        chunks_t &chunks)
{
    request_map_ = true;
    lock_t static_map_lock(static_map_mutex_);
    notify_event_.notify_one();
    notify_static_map_.wait(static_map_lock);
    map = static_map_;
    chunks = allocated_distributions_;
    chunks.insert(chunks.end(), touched_distributions_.begin(), touched_distributions_.end());
    chunks.insert(chunks.end(), untouched_distributions_.begin(), untouched_distributions_.end());
}

void NDTGridMapper::requestMap()
{
    request_map_ = true;
}

void NDTGridMapper::setCallback(const callback_t &cb)
{
    if(!request_map_) {
        callback_ = cb;
    }
}

void NDTGridMapper::loop()
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

void NDTGridMapper::mapRequest()
{
    if(request_map_ && dynamic_map_) {
        cslibs_math_2d::Transform2d origin = dynamic_map_->getOrigin();
        const std::size_t height = static_cast<std::size_t>(dynamic_map_->getHeight() / sampling_resolution_);
        const std::size_t width  = static_cast<std::size_t>(dynamic_map_->getWidth()  / sampling_resolution_);

        static_map_.data().reset(new static_map_t(origin,
                                                  sampling_resolution_,
                                                  height,
                                                  width));
        allocated_distributions_.clear();
        touched_distributions_.clear();
        untouched_distributions_.clear();
        static_map_.stamp() = latest_time_;

        const int chunk_step = static_cast<int>(dynamic_map_->getResolution() / sampling_resolution_);
        const dynamic_map_t::index_t min_distribution_index = dynamic_map_->getMinDistributionIndex();
        const dynamic_map_t::index_t max_distribution_index = dynamic_map_->getMaxDistributionIndex();

        for(int i = min_distribution_index[1] ; i <= max_distribution_index[1] ; ++i) {
            for(int j = min_distribution_index[0] ; j <= max_distribution_index[0] ; ++j) {
                dynamic_map_t::container_handle_t distribution = dynamic_map_->getDistribution({{j,i}});
                if(!distribution.empty()) {
                    const int cx = (j - min_distribution_index[0]) * static_cast<int>(chunk_step);
                    const int cy = (i - min_distribution_index[1]) * static_cast<int>(chunk_step);


                    cslibs_math_2d::Point2d ll(cx * sampling_resolution_, cy * sampling_resolution_);
                    cslibs_math_2d::Point2d ru = ll + cslibs_math_2d::Point2d(chunk_step * sampling_resolution_);
                    switch(distribution->getAction()) {
                    case dynamic_map_t::distribution_container_t::ALLOCATED:
                        allocated_distributions_.emplace_back(cslibs_math_2d::Box2d(origin * ll, origin * ru));
                        break;
                    case dynamic_map_t::distribution_container_t::TOUCHED:
                        touched_distributions_.emplace_back(cslibs_math_2d::Box2d(origin * ll, origin * ru));
                        break;
                    default:
                        untouched_distributions_.emplace_back(cslibs_math_2d::Box2d(origin * ll, origin * ru));
                    }

                    for(int k = 0 ; k < chunk_step ; ++k) {
                        for(int l = 0 ; l < chunk_step ; ++l) {
                            const cslibs_math_2d::Point2d p(j * resolution_ + l * sampling_resolution_,
                                                            i * resolution_ + k * sampling_resolution_);
                            static_map_.data()->at(static_cast<std::size_t>(cx + l),
                                                   static_cast<std::size_t>(cy + k)) = distribution->data().sampleNonNormalized(p);
                        }
                    }
                    distribution->setNone();
                }
            }
        }
        cslibs_gridmaps::static_maps::algorithms::normalize<double>(*static_map_.data());
        callback_(static_map_, allocated_distributions_, touched_distributions_, untouched_distributions_);
    }
    request_map_ = false;
    notify_static_map_.notify_one();
}

void NDTGridMapper::process(const Measurement2d &m)
{
    if(!dynamic_map_) {
        dynamic_map_.reset(new dynamic_map_t(cslibs_math_2d::Transform2d::identity(),
                                             resolution_));
        latest_time_ = m.stamp;
    }

    if(m.stamp > latest_time_) {
        latest_time_ = m.stamp;
    }

    for(const auto &p : *(m.points)) {
        const cslibs_math_2d::Point2d pm = m.origin * p;
        dynamic_map_->add(pm);
    }
}
}
