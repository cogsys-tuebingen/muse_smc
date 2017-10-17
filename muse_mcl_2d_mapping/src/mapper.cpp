#include "mapper.h"

using namespace muse_mcl_2d_mapping;

Mapper::Mapper()
{

}

Mapper::~Mapper()
{
    stop_ = true;
    notify_event_.notify_one();
    if(thread_.joinable())
        thread_.join();
}


void Mapper::insert(const Pointcloud2D::Ptr &points)
{
    q_.emplace(points);
}


void Mapper::loop()
{
    lock_t notify_event_mutex_lock(notify_event_mutex_);
    while(!stop_) {
        notify_event_.wait(notify_event_mutex_lock);
        while(q_.hasElements()) {
            if(stop_)
                break;
            auto e = q_.pop();
            process(e);
        }
    }
}
