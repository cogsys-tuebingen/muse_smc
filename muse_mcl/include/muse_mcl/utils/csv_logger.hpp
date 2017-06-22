#ifndef FILTERSTATELOGGER_HPP
#define FILTERSTATELOGGER_HPP

#include <fstream>
#include <iostream>
#include <sstream>
#include <chrono>
#include <mutex>
#include <thread>
#include <atomic>
#include <condition_variable>
#include <queue>

namespace muse_mcl {
template<typename ... Types>
class CSVLogger {
public:
    using Ptr = std::shared_ptr<CSVLogger<Types ...>>;

    static constexpr std::size_t size = sizeof ... (Types);
    using Header = std::array<std::string, size>;

    inline void log(const Types ... ts)
    {
        long s, ms;
        getTime(s, ms);
        ms += 1000 * s - start_time_;

        std::string string_build = std::to_string(static_cast<double>(ms) / 1e3) + "," +
                    buildString(ts...);

        std::unique_lock<std::mutex> q_lock(q_mutex_);
        q_.push(string_build);
        notify_log_.notify_one();
    }

    CSVLogger(const Header &header,
              const std::string &path = "") :
        header_(header),
        path_(path),
        stop_(false)
    {
        running_ = true;
        worker_thread_ = std::thread([this]{loop();});
        worker_thread_.detach();
    }

    virtual ~CSVLogger()
    {
        if(running_) {
            stop_ = true;
            notify_log_.notify_one();
            if(worker_thread_.joinable())
                worker_thread_.join();
        }
    }

private:
    std::ofstream           out_;
    Header                  header_;
    std::string             path_;
    long                    start_time_;

    std::thread             worker_thread_;
    std::mutex              q_mutex_;
    std::queue<std::string> q_;
    std::mutex              notify_mutex_;
    std::condition_variable notify_log_;

    std::atomic_bool        running_;
    std::atomic_bool        stop_;

    void loop()
    {
        out_.open(path_);
        if(!out_.is_open()) {
            std::stringstream ss;
            long s, ms;
            getTime(s, ms);
            ss << "/tmp/muse_filter_state" << getTime() << ".log";
            out_.open(ss.str());
        }

        if(size > 0) {
            out_ << "time,";
            for(std::size_t i = 0 ; i < size - 1 ; ++i) {
                out_ << header_[i];
            }
            out_ << header_[size - 1];
        } else {
            out_ << "time";
        }
        out_ << std::endl;

        auto dumpQ = [this] () {
            while(!q_.empty()) {
                std::unique_lock<std::mutex> q_lock(q_mutex_);
                auto f = q_.front();
                q_.pop();
                q_lock.unlock();

                out_ << f << std::endl;
            }
        };

        std::unique_lock<std::mutex> notify_lock(notify_mutex_);
        while(!stop_) {
            notify_log_.wait(notify_lock);
            dumpQ();
        }
        dumpQ();

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

    template<typename WT, typename ... WTypes>
    inline std::string buildString(const WT &t, WTypes ... ts) const
    {
        return std::to_string(t) + "," + buildString(ts...);
    }

    template<typename T>
    inline std::string buildString(const T &t) const
    {
        return std::to_string(t);
    }

};

using FilterStateLoggerDefault = CSVLogger<std::size_t, std::size_t, double, double, double>;

}

#endif // FILTERSTATELOGGER_HPP
