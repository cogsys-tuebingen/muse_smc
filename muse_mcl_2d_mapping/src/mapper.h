#ifndef MAPPER_H
#define MAPPER_H

#include <muse_mcl_2d_mapping/pointcloud_2d.hpp>
#include <muse_smc/utility/synchronized_queue.hpp>

#include <atomic>
#include <thread>
#include <condition_variable>
#include <mutex>

namespace muse_mcl_2d_mapping {
class Mapper
{
public:
    using lock_t        = std::unique_lock<std::mutex>;

    Mapper();
    virtual ~Mapper();


    void insert(const Pointcloud2D::Ptr &points);

protected:
    muse_smc::synchronized::queue<Pointcloud2D::Ptr> q_;

    std::thread                                      thread_;
    std::condition_variable                          notify_event_;
    std::mutex                                       notify_event_mutex_;
    std::atomic_bool                                 stop_;

    void loop();
    virtual void process(const Pointcloud2D::Ptr &points) = 0;
};
}

#endif // MAPPER_H
