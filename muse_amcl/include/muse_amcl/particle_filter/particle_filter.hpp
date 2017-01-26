#ifndef PARTICLE_FILTER_HPP
#define PARTICLE_FILTER_HPP

#include "particle_set.hpp"
#include "update_queue.hpp"
#include "propagation_queue.hpp"

#include "../data_sources/tf_provider.hpp"

#include <memory>
#include <thread>
#include <atomic>

namespace muse_amcl {
class ParticleFilter {
public:
    ParticleFilter() :
        tf_provider_(new TFProvider),
        stop_(false),
        running_(false)
    {
    }

    virtual ~ParticleFilter()
    {
        disable();
    }

    /**
     * @brief   Get the TFProvider, which should be used in all backend
     *          threads, therefore in the the update objects.
     * @return  the TFProvider
     */
    TFProvider::Ptr getTFProvider()
    {
        return tf_provider_;
    }

    /**
     * @brief   Return the update queue which is then used by the front end
     *          update managers.
     * @return  the update queue
     */
    UpdateQueue& getUpdateQueue()
    {
        return update_queue_;
    }

    /**
     * @brief   Return the propagation queue.
     * @return  the propagation queue
     */
    PropagationQueue& getPropagationQueue()
    {
        return propagation_queue_;
    }

    /**
     * @brief   Setup the particle filter.
     *          Retrieve all important parameters
     * @param   nh_private
     */
    void setup(ros::NodeHandle &nh_private)
    {

    }

    /**
     * @brief Enable the particle filter thread.
     */
    void enable()
    {
        if(running_)
            return;
        worker_thread_ = std::thread([this](){doExecute();});
        running_ = true;
    }

    /**
     * @brief Disable the particle filter thread.
     */
    void disable()
    {
        stop_ = true;
        update_queue_.clear();
        update_queue_.disableWaiting();
        worker_thread_.join();
        running_ = false;
    }

protected:
    TFProvider::Ptr  tf_provider_;
    UpdateQueue      update_queue_;
    PropagationQueue propagation_queue_;

    ParticleSet::Ptr particle_set_;

    std::thread      worker_thread_;
    std::atomic_bool stop_;
    std::atomic_bool running_;

    void doExecute()
    {
        while(!stop_) {
            /// get next update
            /// check if can be propagated till then
            /// propagate
            /// update
            /// check if it is time to resample
            /// check if we can publish tf
        }
    }

};
}

#endif // PARTICLE_FILTER_HPP
