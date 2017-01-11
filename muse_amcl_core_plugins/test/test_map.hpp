#ifndef TEST_MAP_HPP
#define TEST_MAP_HPP

#include <muse_amcl/data_types/map.hpp>

namespace muse_amcl {
class TestMap : public muse_amcl::Map
{
public:
    TestMap(const std::string &frame,
            const math::Point &min,
            const math::Point &max) :
        Map(frame)
    {
    }

    virtual inline math::Point getMin() const override
    {
        return math::Point(std::numeric_limits<double>::lowest(),
                           std::numeric_limits<double>::lowest(),
                           std::numeric_limits<double>::lowest());
    }

    virtual inline math::Point getMax() const override
    {
        return math::Point(std::numeric_limits<double>::max(),
                           std::numeric_limits<double>::max(),
                           std::numeric_limits<double>::max());
    }

    const math::Point min;
    const math::Point max;
};
}

#endif // TEST_MAP_HPP
