#ifndef FILTERSTATE_PUBLISHER_HPP
#define FILTERSTATE_PUBLISHER_HPP

#include <visualization_msgs/MarkerArray.h>


#include <thread>
#include <atomic>
#include <queue>
#include <condition_variable>

#include <muse_amcl/particle_filter/particle_set.hpp>
#include <muse_amcl/ParticleSetMsg.h>

namespace muse_amcl {
namespace color {
#define __HSV2RGB__(H, S, V, R, G, B) \
    { \
    double _h = H/60.; \
    int _hf = (int)floor(_h); \
    int _hi = ((int)_h)%6; \
    double _f = _h - _hf; \
    \
    double _p = V * (1. - S); \
    double _q = V * (1. - _f * S); \
    double _t = V * (1. - (1. - _f) * S); \
    \
    switch (_hi) \
    { \
    case 0: \
    R = V; G =_t; B = _p; \
    break; \
    case 1: \
    R = _q; G = V; B = _p; \
    break; \
    case 2: \
    R = _p; G = V; B = _t; \
    break; \
    case 3: \
    R = _p; G = _q; B = V; \
    break; \
    case 4: \
    R = _t; G = _p; B = V; \
    break; \
    case 5: \
    R = V; G = _p; B = _q; \
    break; \
} \
}
/// 0 - 120 deg
}






class FilterStatePublisher {
public:
    using Ptr = std::shared_ptr<FilterStatePublisher>;

    /**
     * @brief TransformPublisherAnchored constructor.
     * @param rate          - the publication rate
     * @param odom_frame    - the odometry frame
     * @param base_frame    - the base frame
     * @param world_frame   - the world fram
     */
    FilterStatePublisher(const std::string &world_frame) :
        world_frame_(world_frame),
        running_(false),
        stop_(false),
        nh_private_("~"),
        marker_count_(0),
        max_(std::numeric_limits<double>::lowest())
    {
        pub_markers_ = nh_private_.advertise<visualization_msgs::MarkerArray>("/muse_amcl/markers", 1);
        pub_particles_ = nh_private_.advertise<muse_amcl::ParticleSetMsg>("/muse_amcl/particles", 1);
        worker_thread_ = std::thread([this]{loop();});
        worker_thread_.detach();
    }

    virtual ~FilterStatePublisher()
    {
        if(!running_)
            return;

        stop_ = true;
        notify_.notify_one();
        if(worker_thread_.joinable())
            worker_thread_.join();
    }

    void addState(const ParticleSet::Particles &particles,
                  const ros::Time &time)
    {
        std::unique_lock<std::mutex> q_lock(q_mutex_);
        q_.push(ParticleSet::Particles::Ptr(new ParticleSet::Particles(particles)));
        q_times_.push(time);
        notify_.notify_one();
    }

private:
    const std::string                       world_frame_;

    std::atomic_bool                        running_;
    std::atomic_bool                        stop_;
    std::thread                             worker_thread_;

    std::mutex                              q_mutex_;
    std::queue<ParticleSet::Particles::Ptr> q_;
    std::queue<ros::Time>                   q_times_;
    std::condition_variable                 notify_;

    ros::NodeHandle                         nh_private_;
    ros::Publisher                          pub_markers_;
    ros::Publisher                          pub_particles_;
    std::size_t                             marker_count_;
    double                                  max_;

    inline void loop()
    {
        running_ = true;
        std::unique_lock<std::mutex> q_lock(q_mutex_);
        auto dumpQ = [this, &q_lock] () {
            while(!q_.empty()) {
                auto s = q_.front();
                q_.pop();
                auto t = q_times_.front();
                q_times_.pop();

                q_lock.unlock();
                publisheMarkers(s, t);
                publishParticles(s, t);
                q_lock.lock();
            }
        };

        while(!stop_) {
            notify_.wait(q_lock);
            dumpQ();
        }
        dumpQ();
        running_ = false;
    }

    inline void publisheMarkers (const ParticleSet::Particles::Ptr &samples,
                                 const ros::Time &time)
    {
        visualization_msgs::Marker marker_prototype;
        marker_prototype.header.frame_id = world_frame_;
        marker_prototype.header.stamp = time;
        marker_prototype.ns      = "muse_amcl";
        marker_prototype.type    = visualization_msgs::Marker::ARROW;
        marker_prototype.action  = visualization_msgs::Marker::MODIFY;
        marker_prototype.scale.x = 0.25;
        marker_prototype.scale.y = 0.05;
        marker_prototype.scale.z = 0.05;

        const std::size_t sample_size = samples->size();
        if(sample_size != marker_count_) {
            if(marker_count_ > 0) {
                visualization_msgs::MarkerArray::Ptr marker_array_clear(new visualization_msgs::MarkerArray);
                for(std::size_t i = 0 ; i < sample_size ; ++i) {
                    auto m = marker_prototype;
                    m.action = visualization_msgs::Marker::DELETE;
                    m.id = i;
                    marker_array_clear->markers.push_back(m);
                }
                pub_markers_.publish(marker_array_clear);
            }
        }

        visualization_msgs::MarkerArray::Ptr marker_array_modify(new visualization_msgs::MarkerArray);

        for(std::size_t i = 0 ; i < sample_size ; ++i) {
            if(samples->at(i).weight_ > max_) {
                max_ = samples->at(i).weight_;
            }
        }

        for(std::size_t i = 0 ; i < sample_size ; ++i) {
            visualization_msgs::Marker m = marker_prototype;
            auto &s = samples->at(i);
            m.id = i;
            tf::poseTFToMsg(s.pose_.getPose(), m.pose);

            m.color.a = 1.f;
            if(max_ > 0.0) {
                __HSV2RGB__(120.0 * samples->at(i).weight_ / max_, 1.0, 1.0, m.color.r, m.color.g, m.color.b);
            } else {
                __HSV2RGB__(0.0, 1.0, 1.0, m.color.r, m.color.g, m.color.b);
            }

            marker_array_modify->markers.push_back(m);
        }
        pub_markers_.publish(marker_array_modify);


        marker_count_ = sample_size;
    }

    inline void publishParticles(const ParticleSet::Particles::Ptr &samples,
                                 const ros::Time &time)
    {
        ParticleSetMsg::Ptr msg(new ParticleSetMsg);
        msg->header.stamp = time;
        const std::size_t sample_size = samples->size();
        for(std::size_t i = 0 ; i < sample_size ; ++i) {
            auto &s = samples->at(i);
            ParticleMsg p;
            tf::poseTFToMsg(s.pose_.getPose(), p.pose);
            p.weight.data = s.weight_;
            msg->particles.emplace_back(p);
        }
        pub_particles_.publish(msg);
    }
};
}
#endif // FILTERSTATE_PUBLISHER_HPP
