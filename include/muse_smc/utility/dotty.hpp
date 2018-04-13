#ifndef DOTTY_HPP
#define DOTTY_HPP

#include <fstream>
#include <iostream>
#include <sstream>
#include <chrono>
#include <mutex>
#include <thread>
#include <atomic>
#include <condition_variable>
#include <queue>
#include <map>
#include <cslibs_time/time.hpp>

namespace muse_smc {
class Dotty {
public:
    using Ptr = std::shared_ptr<Dotty>;

    Dotty() :
        path_("/tmp/muse_filter_state_" + getTime()),
        split_(0),
        lines_(0),
        states_(0),
        predictions_(0),
        next_prediction_is_interpolated_(false),
        stop_(false)
    {
        running_ = true;
        worker_thread_ = std::thread([this]{loop();});
    }

    virtual ~Dotty()
    {
        if(running_) {
            stop_ = true;
            notify_log_.notify_one();
            if(worker_thread_.joinable())
                worker_thread_.join();
        }
    }

    void addState(const Time &time)
    {
        std::string state = "state_" + std::to_string(states_);
        std::string node  =  state +
                " [label=\"x_" + std::to_string(states_) +
                " at time \\n" +
                std::to_string(time.seconds()) +
                "\"]";
        {
            std::unique_lock<std::mutex> q_lock(q_mutex_);
            q_.push(node);
            if(states_ > 0) {
                q_.push(last_state_ + " -> " + state);
            }
            last_state_ = state;
            ++states_;
        }
        notify_log_.notify_one();
    }

    void addUpdate(const Time &time, const std::string &name)
    {
        std::string update = "update_" + name + "_" + std::to_string(updates_[name]);
        std::string node = update +
                " [label=\"" + name +
                " at time \\n" +
                std::to_string(time.seconds()) +
                "\"]";
        {
            std::unique_lock<std::mutex> q_lock(q_mutex_);
            q_.push(node);
            if(last_state_ != "") {
                q_.push(update + " -> " + last_state_);
            }
            ++updates_[name];
        }
        notify_log_.notify_one();
    }

    void addPrediction(const Time &time,
                       const bool interpolated = false)
    {
        std::string prediction = "prediction_" + std::to_string(predictions_);
        std::string node  =  prediction +
                " [label=\"u_" + std::to_string(predictions_) +
                " at time \\n" +
                std::to_string(time.seconds()) +
                "\"";
        if(interpolated || next_prediction_is_interpolated_) {
            node += ",color=dodgerblue3]";
        } else {
            node += "]";
        }
        {
            std::unique_lock<std::mutex> q_lock(q_mutex_);
            q_.push(node);
            if(last_state_ != "") {
                q_.push(prediction + " -> " + last_state_);
            }
            ++predictions_;
        }
        notify_log_.notify_one();
        next_prediction_is_interpolated_ = interpolated;
    }

private:
    std::ofstream           out_;
    std::string             path_;
    std::size_t             split_;
    std::size_t             lines_;

    static const std::size_t MAX_LINES = 1000;

    std::map<std::string, std::size_t> updates_;
    std::size_t             states_;
    std::size_t             predictions_;
    std::string             last_state_;
    bool                    next_prediction_is_interpolated_;

    std::thread             worker_thread_;
    std::mutex              q_mutex_;
    std::queue<std::string> q_;
    std::mutex              notify_mutex_;
    std::condition_variable notify_log_;

    std::atomic_bool        running_;
    std::atomic_bool        stop_;

    void loop()
    {
        auto dumpQ = [this] () {
            while(!q_.empty()) {
                std::unique_lock<std::mutex> q_lock(q_mutex_);
                auto f = q_.front();
                q_.pop();
                q_lock.unlock();
                out_ << f << "\n";
                ++lines_;

                if(lines_ >= 1000) {
                    lines_ = 0;
                    writeEnd();
                    reopenOutStream();
                    writeHeader();
                }
            }
        };

        reopenOutStream();
        writeHeader();

        std::unique_lock<std::mutex> notify_lock(notify_mutex_);
        while(!stop_) {
            notify_log_.wait(notify_lock);
            dumpQ();
        }
        dumpQ();
        writeEnd();

        out_.flush();
        if(out_.is_open())
            out_.close();

        running_ = false;
    }

    inline void getTime(long &seconds,
                        long &milliseconds)
    {
        auto now = std::chrono::time_point_cast<std::chrono::milliseconds>(std::chrono::system_clock::now());
        milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
        seconds = milliseconds / 1000;
        milliseconds = milliseconds % 1000;
    }

    inline std::string getTime()
    {
        long s, ms;
        getTime(s, ms);

        const std::string ms_off = ms >= 100 ? "" : (ms >= 10 ? "0" : "00");
        const std::string time = std::to_string(s) + "." + ms_off + std::to_string(ms);
        return time;
    }

    inline std::string toString(const Time &time)
    {
        long milli = std::floor(time.nanoseconds() / 1e6);
        long s  = milli / 1000;
        long ms = milli % 1000;
        const std::string ms_off = ms >= 100 ? "" : (ms >= 10 ? "0" : "00");
        return std::to_string(s) + "." + ms_off + std::to_string(ms);
    }

    inline void reopenOutStream()
    {
        std::string path = path_ + "_" + std::to_string(split_) + ".dot";
        if(out_.is_open())
            out_.close();
        out_.open(path);
        ++split_;
    }

    inline void writeHeader()
    {
        out_ << "digraph states {" << "\n";
    }

    inline void writeEnd()
    {
        out_ << "}" << "\n";
    }
};
}

#endif // DOTTY_HPP
