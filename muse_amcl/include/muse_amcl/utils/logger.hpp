#ifndef LOGGER_H
#define LOGGER_H

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
class Logger {
public:
    enum Level {NONE = -1, ALL = 0, WARN = 1, ERROR = 2};
    const std::size_t ms_digits = 3;

    inline void info(const std::string &msg,
                     const std::string &sender = "")
    {
        if(level_ == ALL) {
            std::string str = "[" + getTime() + "][INFO]";
            if(sender != "")
                str += "[" + sender + "]";
            else
                str += ": ";
            str += msg;

            std::unique_lock<std::mutex> l(q_mutex_);
            q_.push(str);
            notify_log_.notify_one();
        }
    }

    inline void error(const std::string &msg,
                      const std::string &sender = "")
    {
        if(level_ > 0) {
            std::string str = "[" + getTime() + "][ERROR]";
            if(sender != "")
                str += "[" + sender + "]";
            else
                str += ": ";
            str += msg;

            std::unique_lock<std::mutex> l(q_mutex_);
            q_.push(str);
            notify_log_.notify_one();
        }
    }

    inline void warn(const std::string &msg,
                     const std::string &sender = "")
    {
        if(level_ > 0 && level_ <= WARN) {
            std::string str = "[" + getTime() + "][WARN]" ;
            if(sender != "")
                str += "[" + sender + "]";
            else
                str += ": ";
            str += msg;

            std::unique_lock<std::mutex> l(q_mutex_);
            q_.push(str);
            notify_log_.notify_one();
        }
    }

    inline void markNewLogSection()
    {
        if(level_ > 0) {
            std::unique_lock<std::mutex> l(q_mutex_);
            q_.push(std::string(80, '.'));
            notify_log_.notify_one();
        }
    }

    static inline Logger& getLogger(const Level level = ALL,
                                    const bool write_to_disk = true) {
        static Logger l(level, write_to_disk);
        return l;
    }

private:
    std::atomic_bool        running_;
    std::atomic_bool        stop_;
    std::thread             worker_thread_;

    Level                   level_;
    bool                    write_to_disk_;

    std::mutex              q_mutex_;
    std::queue<std::string> q_;
    std::mutex              notify_mutex_;
    std::condition_variable notify_log_;

    std::ofstream           out_;

    inline Logger(const Level level = ALL,
                  const bool write_to_disk = true) :
        running_(false),
        stop_(false),
        level_(level),
        write_to_disk_(write_to_disk)
    {
        worker_thread_ = std::thread([this](){loop();});
    }


    virtual ~Logger()
    {
        if(running_) {
            stop_ = true;
            notify_log_.notify_one();
            if(worker_thread_.joinable())
                worker_thread_.join();
        }
    }

    void loop()
    {
        running_ = true;

        std::stringstream ss;
        ss << "/tmp/muse_" << getTime() << ".log";
        std::cout << "[Logger]: Log path '" << ss.str() << "'" << std::endl;
        out_.open(ss.str());

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





};
}
#endif // LOGGER_H
