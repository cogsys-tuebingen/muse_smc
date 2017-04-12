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
    virtual ~Logger()
    {
        if(out_.is_open())
            out_.close();
    }

    enum Level {ALL = 0, WARN = 1, ERROR = 2};
    const std::size_t ms_digits = 3;

    inline void info(const std::string &msg,
                     const std::string &sender = "")
    {
        if(enabled_ && level_ == ALL) {
            std::string time;
            getTime(time);

            std::unique_lock<std::mutex> l(mutex_);
            if(sender == "") {
                std::cout << "[" << time << "][INFO] " << msg << std::endl;
                if(write_to_disk_) {
                    out_ << "[" << time << "][INFO] " << msg << std::endl;
                }
            } else {
                std::cout << "[" << time << "][INFO][" << sender << "]:" << msg << std::endl;
                if(write_to_disk_) {
                    out_ << "[" << time << "][INFO][" << sender << "]:" << msg << std::endl;
                }
            }
        }
    }

    inline void error(const std::string &msg,
                      const std::string &sender = "")
    {
        if(enabled_) {
            std::string time;
            getTime(time);

            std::unique_lock<std::mutex> l(mutex_);
            if(sender == "") {
                std::cerr << "[" << time << "][ERROR] " << msg << std::endl;
                if(write_to_disk_)
                    out_ << "[" << time << "][ERROR] " << msg << std::endl;
            } else {
                std::cerr << "[" << time << "]ERROR][" << sender << "]:" << msg << std::endl;
                if(write_to_disk_)
                    out_ << "[" << time << "][ERROR][" << sender << "]:" << msg << std::endl;
            }
        }
    }

    inline void warn(const std::string &msg,
                     const std::string &sender = "")
    {
        if(enabled_ && level_ <= WARN) {
            std::string time;
            getTime(time);

            std::unique_lock<std::mutex> l(mutex_);
            if(sender == "") {
                std::cout << "[" << time << "][WARN] " << msg << std::endl;
                if(write_to_disk_)
                    out_ << "[" << time << "][WARN] " << msg << std::endl;
            } else {
                std::cout << "[" << time << "][WARN][" << sender << "]:" << msg << std::endl;
                if(write_to_disk_)
                    out_ << "[" << time << "][WARN][" << sender << "]:" << msg << std::endl;
            }
        }
    }

    inline void markNewLogSection()
    {
        if(enabled_) {
            std::string line = "";
            for(std::size_t i = 0 ; i < 80 ; ++i)
                line += "-";

            std::unique_lock<std::mutex> l(mutex_);
            std::cout << line << std::endl;
            if(write_to_disk_) {
                out_ << line << std::endl;
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
    bool          enabled_;
    Level         level_;
    bool          write_to_disk_;
    std::ofstream out_;
    std::mutex    mutex_;


    inline void getTime(long &seconds,
                        long &milliseconds)
    {
        auto now = std::chrono::time_point_cast<std::chrono::milliseconds>(std::chrono::system_clock::now());
        seconds = std::chrono::duration_cast<std::chrono::seconds>(now.time_since_epoch()).count();
        milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count()
                - 1000 * seconds;
    }

    inline void getTime(std::string &time)
    {
        long s, ms;
        getTime(s, ms);

        const std::string ms_off = ms >= 100 ? "" : (ms >= 10 ? "00" : "0");
        time = std::to_string(s) + "." + ms_off + std::to_string(ms);
    }


    inline Logger(const bool  enable = false,
                  const Level level = ALL,
                  const bool write_to_disk = true) :
        enabled_(enable),
        level_(level),
        write_to_disk_(write_to_disk)
    {

        std::stringstream ss;
        std::string time;
        getTime(time);

        ss << "/tmp/muse_" << time << ".log";
        std::cout << "[Logger]: Log path '" << ss.str() << "'" << std::endl;
        out_.open(ss.str());
    }


};
}
#endif // LOGGER_H
