#ifndef TEST_MAP_PROVIDER_HPP
#define TEST_MAP_PROVIDER_HPP

#include "test_map.hpp"
#include <muse_mcl/data_sources/map_provider.hpp>

namespace muse_mcl {
class TestMapProvider : public MapProvider {
public:
    typedef std::shared_ptr<TestMapProvider> Ptr;

    TestMapProvider(const std::string name,
                    const TestMap::ConstPtr &map)
    {
        name_ = name;
        map_  = map;
    }

    virtual Map::ConstPtr getMap() const override
    {
        return map_;
    }

private:
    Map::ConstPtr map_;

    virtual void doSetup(ros::NodeHandle &nh_private)
    {
    }

};
}


#endif // TEST_MAP_PROVIDER_HPP
