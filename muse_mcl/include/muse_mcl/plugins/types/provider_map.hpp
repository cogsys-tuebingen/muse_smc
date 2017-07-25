#ifndef MAP_PROVIDER_HPP
#define MAP_PROVIDER_HPP

#include <memory>
#include <muse_mcl/data_types/map.hpp>
#include <ros/ros.h>


namespace muse_mcl {
class ProviderMap {
public:
    typedef std::shared_ptr<ProviderMap> Ptr;

    virtual ~ProviderMap()
    {
    }

    inline const static std::string Type()
    {
        return "muse_mcl::ProviderMap";
    }

    inline std::string getName() const
    {
        return name_;
    }

    inline void setup(const std::string &name,
                      ros::NodeHandle   &nh_private)
    {
            name_ = name;
            doSetup(nh_private);
    }

    virtual Map::ConstPtr getMap() const = 0;

protected:
    std::string  name_;

    virtual void doSetup(ros::NodeHandle &nh_private) = 0;

    std::string privateParameter(const std::string &name)
    {
        return name_ + "/" + name;
    }

};
}

#endif /* MAP_PROVIDER_HPP */
