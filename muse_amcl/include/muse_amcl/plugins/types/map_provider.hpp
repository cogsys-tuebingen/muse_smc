#pragma once

#include <memory>

#include "../data/map.hpp"

namespace muse_amcl {
class MapProvider {
public:
    typedef std::shared_ptr<MapProvider> Ptr;

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

protected:
    std::string name_;

    virtual void loadParameters(ros::NodeHandle &nh_private) = 0;

    std::string param(const std::string &name)
    {
        return name_ + "/" + name;
    }

};
}
