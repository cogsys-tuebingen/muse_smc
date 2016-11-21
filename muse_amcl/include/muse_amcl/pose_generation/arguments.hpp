#pragma once

#include <array>
#include <typeinfo>
#include "../math/angle.hpp"

namespace muse_amcl {
namespace pose_generation {
struct Radian {
    static inline bool requiresNormalization()
    {
        return true;
    }

    static inline double normalize(const double rad)
    {
        return math::angle::normalize(rad);
    }
};

struct Metric {
    static inline bool requiresNoramlization()
    {
        return false;
    }
};

template<typename... T>
struct is_valid_type : std::false_type { };

template<>
struct is_valid_type<> : std::true_type { };

template<>
struct is_valid_type<Radian> : std::true_type { };

template<>
struct is_valid_type<Metric> : std::true_type { };

template<typename T1, typename T2, typename... Ts>
struct is_valid_type<T1, T2, Ts...> : is_valid_type<T2, Ts...> { };

}
}
