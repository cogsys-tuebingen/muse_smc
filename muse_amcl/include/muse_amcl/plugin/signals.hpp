#pragma once

#include <memory>
#include <functional>
#include <map>


template<typename Slot>
class Signal;

template<typename Slot>
class Signal : public std::enable_shared_from_this<Signal<Slot>> {
public:
    typedef std::shared_ptr<Signal<Slot>> Ptr;

    class Connection {
    public:
        typedef std::shared_ptr<Connection> Ptr;

        Connection(Signal<Slot> &_signal) :
            signal(_signal)
        {
        }

        virtual ~Connection()
        {
            signal.disconnect(this);
        }

    private:
        Signal &signal;
    };


    template<typename Function>
    typename Connection::Ptr connect(Function &_f)
    {
        typename Connection::Ptr c(new Signal::Connection(*this));
        connections[c.get()] = _f;
        return c;
    }

    void disconnect(typename Connection::Ptr &_c)
    {
        connections.erase(_c.get());
    }

    void disconnect(Connection *_c)
    {
        connections.erase(_c);
    }

    template<typename... Args>
    void operator ()(Args... args)
    {
        for(auto &c : connections) {
            c.second(args...);
        }
    }

private:
    std::map<Connection*, Slot> connections;

};


