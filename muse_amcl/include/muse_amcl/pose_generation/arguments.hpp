#pragma once

#include <array>
#include <typeinfo>
#include "../math/angle.hpp"

namespace muse_amcl {
namespace pose_generation {
/**
 * @brief The Radian struct represents a semantic type,
 *        Radian values should be normalized.
 */
struct Radian {
    /**
     * @brief Check if this semantic type needs normalization.
     * @return
     */
    static inline bool requiresNoramlization()
    {
        return true;
    }

    /**
     * @brief Apply semantic specific normalization.
     * @param rad - radian value
     */
    static inline void normalize(double &rad)
    {
       rad = math::angle::normalize(rad);
    }
};

/**
 * @brief The Metric struct represents a semantic type.
 *        Generally metric values do not have to be normalized.
 */
struct Metric {
    /**
     * @brief Check if this semantic type needs normalization.
     * @return
     */
    static inline bool requiresNoramlization()
    {
        return false;
    }

    /**
     * @brief The normalization in this case is the identity.
     */
    static inline void normalize(double &)
    {
    }
};

/**
 * Semantic argument type traits.
 */
template<unsigned int N, typename Array, typename... Ts>
struct Arguments;

/**
 * The argument list for the pose generators must be of size one at least.
 * This it the terminating struct, applying the normalization to the last
 * type.
 */
template<typename Array, typename T, typename... Ts>
struct Arguments<1, Array, T, Ts...>
{
    typedef T Type;
    inline static void normalize(Array &arr)
    {
        if(T::requiresNoramlization()) {
            T::normalize(arr[Array::RowsAtCompileTime - 1]);
        }
    }
};

/**
 * This the general template argument processing struct.
 * It applies the normalization to the (N - Dim)-th type argument.
 */
template<unsigned int N, typename Array, typename T, typename... Ts>
struct Arguments<N, Array, T, Ts...>
{
    static_assert(N > 0, "Dimension of arguments must be at least 1!");

    inline static void normalize(Array &arr)
    {
        if(T::requiresNoramlization()) {
            T::normalize(arr[Array::RowsAtCompileTime - N]);
        }
        Arguments<N-1, Array, Ts...>::normalize(arr);
    }

};

/**
 * These are the type traits for allowed semantic types.d
 */
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
