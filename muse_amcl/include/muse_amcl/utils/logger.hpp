#ifndef LOGGER_H
#define LOGGER_H

#include <fstream>
#include <iostream>
#include <sstream>
#include <chrono>
#include <mutex>

namespace muse_amcl {
class Logger {
public:
    enum Level {ALL = 0, WARN = 1, ERROR = 2};
    const std::size_t ms_digits = 3;

    void info(const std::string &msg,
              const std::string &sender = "")
    {
        if(enabled_ && level_ == ALL) {
            long s, ms;
            getTime(s,ms);

            const std::string ms_off = ms >= 100 ? "" : (ms >= 10 ? "0" : "00");
            std::unique_lock<std::mutex> l(mutex_);
            if(sender == "") {
                std::cout << "[" << s << "." << ms_off << ms << "][INFO] " << msg << std::endl;
                if(write_to_disk_) {
                    out_ << "[" << s  << "." << ms_off << ms << "][INFO] " << msg << std::endl;
                }
            } else {
                std::cout << "[" << s << "." << ms_off << ms << "][INFO][" << sender << "]:" << msg << std::endl;
                if(write_to_disk_) {
                    out_ << "[" << s << "." << ms_off << ms << "][INFO][" << sender << "]:" << msg << std::endl;
                }
            }
        }
    }

    void error(const std::string &msg,
               const std::string &sender = "")
    {
        if(enabled_) {
            long s, ms;
            getTime(s,ms);

            const std::string ms_off = ms >= 100 ? "" : (ms >= 10 ? "00" : "0");
            std::unique_lock<std::mutex> l(mutex_);
            if(sender == "") {
                std::cerr << "[" << s << "." << ms_off << ms << "][ERROR] " << msg << std::endl;
                if(write_to_disk_)
                    out_ << "[" << s << "." << ms_off << ms << "][ERROR] " << msg << std::endl;
            } else {
                std::cerr << "[" << s << "." << ms_off << ms << "]ERROR][" << sender << "]:" << msg << std::endl;
                if(write_to_disk_)
                    out_ << "[" << s << "." << ms_off << ms << "][ERROR][" << sender << "]:" << msg << std::endl;
            }
        }
    }

    void warn(const std::string &msg,
              const std::string &sender = "")
    {
        if(enabled_ && level_ <= WARN) {
            long s, ms;
            getTime(s,ms);

            const std::string ms_off = ms >= 100 ? "" : (ms >= 10 ? "00" : "0");
            std::unique_lock<std::mutex> l(mutex_);
            if(sender == "") {
                std::cout << "[" << s << "." << ms_off << ms << "][WARN] " << msg << std::endl;
                if(write_to_disk_)
                    out_ << "[" << s << "." << ms_off << ms << "][WARN] " << msg << std::endl;
            } else {
                std::cout << "[" << s << "." << ms_off << ms << "][WARN][" << sender << "]:" << msg << std::endl;
                if(write_to_disk_)
                    out_ << "[" << s << "." << ms_off << ms << "][WARN][" << sender << "]:" << msg << std::endl;
            }
        }
    }

    static inline Logger& getLogger(const bool enable = false,
                                    const Level level = ALL,
                                    const bool write_to_disk = true) {
        static Logger l(enable, level, write_to_disk);
        return l;
    }

private:
    virtual ~Logger()
    {
        if(out_.is_open())
            out_.close();
    }

    inline void getTime(long &seconds,
                        long &milliseconds)
    {
        auto now = std::chrono::time_point_cast<std::chrono::milliseconds>(std::chrono::system_clock::now());
        seconds = std::chrono::duration_cast<std::chrono::seconds>(now.time_since_epoch()).count();
        milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count()
                - 1000 * seconds;
    }

    Logger(const bool  enable = false,
           const Level level = ALL,
           const bool write_to_disk = true) :
        enabled_(enable),
        level_(level),
        write_to_disk_(write_to_disk)
    {

        std::stringstream ss;
        long s, ms;
        getTime(s, ms);
        ss << "/tmp/muse_" << s << "." << ms << ".log";
        std::cout << "[Logger]: Log path '" << ss.str() << "'" << std::endl;
        out_.open(ss.str());
    }

    bool          enabled_;
    Level         level_;
    bool          write_to_disk_;
    std::ofstream out_;
    std::mutex    mutex_;

};
}
#endif // LOGGER_H
