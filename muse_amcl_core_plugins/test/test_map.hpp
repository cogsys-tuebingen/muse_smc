#ifndef TEST_MAP_HPP
#define TEST_MAP_HPP

#include <muse_amcl/data_types/map.hpp>

namespace muse_mcl {
class TestMap : public muse_mcl::Map
{
public:
    TestMap(const std::string &frame,
            const math::Point &min,
            const math::Point &max) :
        Map(frame),
        min_(min),
        max_(max)
    {
    }

    virtual inline math::Point getMin() const override
    {
        return min_;
    }

    virtual inline math::Point getMax() const override
    {
        return max_;
    }

    virtual inline bool validate(const math::Pose &p) const override
    {
        bool dim_x = p.x() >= min_.x() && p.x() <= max_.x();
        bool dim_y = p.y() >= min_.y() && p.y() <= max_.y();
        return dim_x && dim_y;
    }


    const math::Point min_;
    const math::Point max_;
};
}

#endif // TEST_MAP_HPP
