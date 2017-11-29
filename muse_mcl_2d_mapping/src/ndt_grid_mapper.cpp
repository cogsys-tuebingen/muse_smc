#include "ndt_grid_mapper.h"

namespace muse_mcl_2d_mapping {
NDTGridMapper::NDTGridMapper(const double resolution,
                             const double sampling_resolution,
                             const std::string &frame_id) :
    stop_(false),
    request_map_(false),
    callback_([](const static_map_t::Ptr &){}),
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
        static_map_.stamp() = latest_time_;

        const double bundle_resolution = dynamic_map_->getBundleResolution();
        const int chunk_step = static_cast<int>(bundle_resolution / sampling_resolution_);
        const dynamic_map_t::index_t min_distribution_index = dynamic_map_->getMinDistributionIndex();
        const dynamic_map_t::index_t max_distribution_index = dynamic_map_->getMaxDistributionIndex();

        auto sample = [](const dynamic_map_t::distribution_t *d,
                         const cslibs_math_2d::Point2d &p) {
            return d ? d->data().sampleNonNormalized(p) : 0.0;
        };
        auto sample_bundle = [&sample] (const dynamic_map_t::distribution_bundle_t* b,
                                        const cslibs_math_2d::Point2d &p)
        {
            return 0.25 * (sample(b->at(0), p) +
                           sample(b->at(1), p) +
                           sample(b->at(2), p) +
                           sample(b->at(3), p));
        };

        for(int i = min_distribution_index[1] ; i <= max_distribution_index[1] ; ++i) {
            for(int j = min_distribution_index[0] ; j <= max_distribution_index[0] ; ++j) {
                dynamic_map_t::distribution_bundle_t* bundle = dynamic_map_->getDistributionBundle({{j,i}});
                if(bundle) {
                    const int cx = (j - min_distribution_index[0]) * static_cast<int>(chunk_step);
                    const int cy = (i - min_distribution_index[1]) * static_cast<int>(chunk_step);
                    for(int k = 0 ; k < chunk_step ; ++k) {
                        for(int l = 0 ; l < chunk_step ; ++l) {
                            const cslibs_math_2d::Point2d p(j * bundle_resolution + l * sampling_resolution_,
                                                            i * bundle_resolution + k * sampling_resolution_);
                            static_map_.data()->at(static_cast<std::size_t>(cx + l),
                                                   static_cast<std::size_t>(cy + k)) = sample_bundle(bundle, p);
                        }
                    }
                }
            }
        }
        cslibs_gridmaps::static_maps::algorithms::normalize<double>(*static_map_.data());
        callback_(static_map_);
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
