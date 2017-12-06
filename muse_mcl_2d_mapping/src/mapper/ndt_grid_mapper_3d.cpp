#include <muse_mcl_2d_mapping/mapper/ndt_grid_mapper_3d.h>

namespace muse_mcl_3d_mapping {
NDTGridMapper3d::NDTGridMapper3d(
        const double        resolution,
        const double        sampling_resolution,
        const std::string & frame_id) :
        stop_(false),
        request_map_(false),
        callback_([](const static_map_t::Ptr &){}),
        resolution_(resolution),
        sampling_resolution_(sampling_resolution),
        frame_id_(frame_id)

{
    thread_ = std::thread([this](){loop();});
}

NDTGridMapper3d::~NDTGridMapper3d()
{
    stop_ = true;
    notify_event_.notify_one();
    if(thread_.joinable())
        thread_.join();
}

void NDTGridMapper3d::insert(const measurement_t &measurement)
{
    q_.emplace(measurement);
    notify_event_.notify_one();
}

void NDTGridMapper3d::get(static_map_stamped_t &map)
{
    request_map_ = true;
    lock_t static_map_lock(static_map_mutex_);
    notify_event_.notify_one();
    notify_static_map_.wait(static_map_lock);
    map = static_map_;
}

void NDTGridMapper3d::requestMap()
{
    request_map_ = true;
}

void NDTGridMapper3d::setCallback(
        const callback_t & cb)
{
    if(!request_map_)
        callback_ = cb;
}

void NDTGridMapper3d::loop()
{
    lock_t notify_event_mutex_lock(notify_event_mutex_);
    while(!stop_) {
        notify_event_.wait(notify_event_mutex_lock);
        while(q_.hasElements()) {
            if(stop_)
                break;

            //mapRequest();
            auto m = q_.pop();
            process(m);
        }
        mapRequest();
    }
}

void NDTGridMapper3d::mapRequest()
{
    cslibs_time::Time now = cslibs_time::Time::now();

    if(request_map_ && dynamic_map_) {
        dynamic_map_t::transform_t origin = dynamic_map_->getOrigin();
        const std::size_t height = static_cast<std::size_t>(dynamic_map_->getHeight() / sampling_resolution_);
        const std::size_t width  = static_cast<std::size_t>(dynamic_map_->getWidth()  / sampling_resolution_);

        static_map_.data().reset(new static_map_t());
        static_map_.stamp() = latest_time_;

        const double bundle_resolution = dynamic_map_->getBundleResolution();
        const int chunk_step = static_cast<int>(bundle_resolution / sampling_resolution_);
        const dynamic_map_t::index_t min_distribution_index = dynamic_map_->getMinDistributionIndex();
        const dynamic_map_t::index_t max_distribution_index = dynamic_map_->getMaxDistributionIndex();

        auto sample = [](const dynamic_map_t::distribution_t * d,
                         const dynamic_map_t::point_t        & p) {
            return d ? d->data().sampleNonNormalized(p) : 0.0;
        };
        auto sample_bundle = [&sample] (const dynamic_map_t::distribution_bundle_t * b,
                                        const dynamic_map_t::point_t               & p)
        {
            return 0.125 * (sample(b->at(0), p) +
                            sample(b->at(1), p) +
                            sample(b->at(2), p) +
                            sample(b->at(3), p) +
                            sample(b->at(4), p) +
                            sample(b->at(5), p) +
                            sample(b->at(6), p) +
                            sample(b->at(7), p));
        };

        for (int h = min_distribution_index[2] ; h <= max_distribution_index[2] ; ++ h) {
            for(int i = min_distribution_index[1] ; i <= max_distribution_index[1] ; ++ i) {
                for(int j = min_distribution_index[0] ; j <= max_distribution_index[0] ; ++ j) {
                    dynamic_map_t::distribution_bundle_t * bundle = dynamic_map_->getDistributionBundle({{j,i,h}});
                    if(bundle) {
                        //*
                        const dynamic_map_t::point_t p(j * bundle_resolution,
                                                       i * bundle_resolution,
                                                       h * bundle_resolution);
                        pcl::PointXYZI prob;
                        prob.x = p(0);
                        prob.y = p(1);
                        prob.z = p(2);
                        prob.intensity = sample_bundle(bundle, p);
                        if (prob.intensity > 0.0 && prob.intensity <= 1.0)
                            static_map_.data()->points.emplace_back(prob);
                        /*/
                        for (int m = 0; m < chunk_step; ++ m) {
                            for(int k = 0 ; k < chunk_step ; ++ k) {
                                for(int l = 0 ; l < chunk_step ; ++ l) {
                                    const dynamic_map_t::point_t p(j * bundle_resolution + l * sampling_resolution_,
                                                                   i * bundle_resolution + k * sampling_resolution_,
                                                                   h * bundle_resolution + m * sampling_resolution_);
                                    pcl::PointXYZI prob;
                                    prob.x = p(0);
                                    prob.y = p(1);
                                    prob.z = p(2);
                                    prob.intensity = sample_bundle(bundle, p);
                                    if (prob.intensity > 0.0 && prob.intensity <= 1.0)
                                        static_map_.data()->push_back(prob);
                                }
                            }
                        }
                        //*/
                    }
                }
            }
        }
//        cslibs_gridmaps::static_maps::algorithms::normalize<double>(*static_map_.data());
        callback_(static_map_);

        std::cout << "Visualization: " << (cslibs_time::Time::now() - now) << std::endl;
    }
    request_map_ = false;
    notify_static_map_.notify_one();
}

void NDTGridMapper3d::process(const measurement_t &m)
{
    if (!dynamic_map_) {
        dynamic_map_.reset(new dynamic_map_t(dynamic_map_t::transform_t::identity(),
                                             resolution_));
        latest_time_ = m.stamp;
    }

    if (m.stamp > latest_time_)
        latest_time_ = m.stamp;

    for (const auto & p : *(m.points)) {
        const dynamic_map_t::point_t pm = m.origin * p;
        if (std::isnormal(pm(0)) && std::isnormal(pm(1)) && std::isnormal(pm(2)))
            dynamic_map_->add(pm);
    }
}
}
