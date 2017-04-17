#ifndef FILTERSTATELOGGER_HPP
#define FILTERSTATELOGGER_HPP

#include <fstream>
#include <iostream>
#include <sstream>
#include <chrono>
#include <mutex>

namespace muse_amcl {
template<typename ... Types>
class FilterStateLogger {
public:
    static constexpr std::size_t size = sizeof ... (Types);
    using Header = std::array<std::string, size>;

    virtual ~FilterStateLogger()
    {
        if(out_.is_open())
            out_.close();
    }

    inline void writeState(const Types ... ts)
    {
//        std::unique_lock<std::mutex> l(mutex_);

//        long s, ms;
//        getTime(s, ms);
//        ms += 1000 * s - start_time_;

//        out_ << static_cast<double>(ms) / 1e3 << ",";
//        write(ts...);
    }

    static inline FilterStateLogger& getLogger(const Header &header = Header(),
                                               const bool relative_time = true) {
        static FilterStateLogger l(header, relative_time);
        return l;
    }

private:
    std::mutex    mutex_;
    std::ofstream out_;
    long          start_time_;

    FilterStateLogger(const Header &header,
                      const bool relative_time) :
        start_time_(0)
    {
        std::stringstream ss;
        long s, ms;
        getTime(s, ms);
        ss << "/tmp/muse_filter_state" << s << "." << ms << ".log";
        out_.open(ss.str());

        if(size > 0) {
            out_ << "time,";
            for(std::size_t i = 0 ; i < size - 1 ; ++i) {
                out_ << header[i];
            }
            out_ << header[size - 1];
        } else {
            out_ << "time";
        }
        out_ << std::endl;

        if(relative_time) {
            start_time_ = ms + 1000 * s;
        }
    }


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

    template<typename WT, typename ... WTypes>
    void write(const WT &t, WTypes ... ts)
    {
        out_ << t << ",";
        write(ts...);
    }

    template<typename T>
    void write(const T &t)
    {
        out_ << t << "\n";
    }

};

using FilterStateLoggerDefault = FilterStateLogger<std::size_t, std::size_t, double, double, double, double>;

}

#endif // FILTERSTATELOGGER_HPP
