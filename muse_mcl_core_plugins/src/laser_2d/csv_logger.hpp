#ifndef CSVLOGGER_H
#define CSVLOGGER_H

#include "registered_frame.hpp"

#include <queue>
#include <mutex>
#include <atomic>
#include <thread>
#include <fstream>
#include <chrono>
#include <condition_variable>
#include <iostream>

class CSVLogger {
public:
    CSVLogger() :
        stop(false)
    {
        thread = std::thread([this](){loop();});
    }

    virtual ~CSVLogger()
    {
        stop = true;
        notify.notify_one();

        if(thread.joinable())
            thread.join();

    }

    void add(const RegisteredFrame::Ptr &frame)
    {
        std::unique_lock<std::mutex> l(mutex);
        frames.push(frame);
        notify.notify_one();
    }

private:
    std::atomic_bool                 stop;
    std::atomic_bool                 running;

    std::condition_variable          notify;
    std::thread                      thread;

    std::mutex                       mutex;
    std::queue<RegisteredFrame::Ptr> frames;

    void loop()
    {
        std::unique_lock<std::mutex> l(mutex);
        while(!stop) {
            notify.wait(l);

            if(stop)
                break;
            dumpQ(l);
        }
    }

    void dumpQ(std::unique_lock<std::mutex> &lock)
    {
        while(!frames.empty()) {
            RegisteredFrame::Ptr f = frames.front();
            frames.pop();
            lock.unlock();
            write(f);
            if(stop)
                return;
            lock.lock();
        }
    }

    inline void write(const RegisteredFrame::Ptr &frame)
    {
        std::string file("/tmp/" + getTime() + ".csv");
        std::ofstream  out(file);
        if(out.is_open()) {
            out << frame->to_string();
            out.flush();
            out.close();
        } else {
            std::cerr << "Error, cannot open '" << file << "'!" << std::endl;
        }
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

        const std::string ms_off = ms >= 100 ? "" : (ms >= 10 ? "00" : "0");
        return std::to_string(s) + "." + ms_off + std::to_string(ms);
    }



};

#endif // CSVLOGGER_H
