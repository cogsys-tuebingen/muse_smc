#pragma once

#include <memory>

#include <muse_amcl/signals/signals.hpp>
#include "../data/map.hpp"

namespace muse_amcl {
class MapProvider {
public:
    typedef std::shared_ptr<MapProvider> Ptr;
    typedef std::function<void()>        Callback;
    typedef Signal<Callback>             DataSignal;
    typedef DataSignal::Connection       DataConnection;

    virtual ~MapProvider()
    {
    }

    inline const static std::string Type()
    {
        return "muse_amcl::MapProvider";
    }

    inline std::string name() const
    {
        return name_;
    }

    inline void setup(const std::string &name,
               ros::NodeHandle   &nh_private)
    {
            name_ = name;
            loadParameters(nh_private);
    }

    virtual Map::ConstPtr map() const = 0;

    DataConnection::Ptr connect(const Callback &callback)
    {
        return map_loaded_.connect(callback);
    }

    void enable()
    {
        map_loaded_.enable();
    }

    void disable()
    {
        map_loaded_.disable();
    }

protected:
    std::string name_;
    DataSignal  map_loaded_;

    virtual void loadParameters(ros::NodeHandle &nh_private) = 0;

    std::string param(const std::string &name)
    {
        return name_ + "/" + name;
    }

};
}
