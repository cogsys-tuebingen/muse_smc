#ifndef INDEX_HPP
#define INDEX_HPP

#include <array>
#include <algorithm>
#include <assert.h>

namespace muse_mcl {
namespace math {

/**
 * @brief The Index struct is an extened version of the std::array<int, Dim>.
 *        It allows minimum, maximum calculation alongside primitive mathematic
 *        functions as addition and subtration.
 */
template<std::size_t _Nm>
struct Index : public std::array<int, _Nm>
{
    using Base = std::array<int, _Nm>;

    /**
     * @brief Index default constructor.
     * @param i - constant value for each field in the array.
     */
    Index(const int i = 0)
    {
        Base::fill(i);
    }

    /**
     * @brief Index constructor by std::arra<int, Dim>.
     * @param other - the other index
     */
    Index(const std::array<int, _Nm> &arr) :
        std::array<int, _Nm>(arr)
    {
    }

    void operator = (const int i)
    {
        Base::fill(i);
    }

    void operator = (const Base &other)
    {
        for(std::size_t __n = 0 ; __n < _Nm ; ++__n) {
            Base::_AT_Type::_S_ref(Base::_M_elems, __n) =
                Base::_AT_Type::_S_ref(other._M_elems, __n);
        }
    }

    /**
     * @brief max sets the maximum imperatively.
     * @param other - the other index
     */
    inline void max(const Index &other)
    {
        for(std::size_t __n = 0 ; __n < _Nm ; ++__n) {
            Base::_AT_Type::_S_ref(Base::_M_elems, __n) =
                    std::max(Base::_AT_Type::_S_ref(Base::_M_elems, __n),
                             Base::_AT_Type::_S_ref(other._M_elems, __n));
        }
    }

    /**
     * @brief min sets the minimum imperatively.
     * @param other - the other index
     */
    inline void min(const Index &other)
    {
        for(std::size_t __n = 0 ; __n < _Nm ; ++__n) {
            Base::_AT_Type::_S_ref(Base::_M_elems, __n) =
                    std::min(Base::_AT_Type::_S_ref(Base::_M_elems, __n),
                             Base::_AT_Type::_S_ref(other._M_elems, __n));
        }
    }

    /**
     * @brief max is the applicative version of building the maximum.
     * @param a - first index
     * @param b - second index
     * @return the maximum
     */
    static inline Index max(const Index &a,
                            const Index &b)
    {
        Index r;
        for(std::size_t __n = 0 ; __n < _Nm ; ++__n) {
            Base::_AT_Type::_S_ref(r._M_elems, __n) =
                    std::max(Base::_AT_Type::_S_ref(a._M_elems, __n),
                             Base::_AT_Type::_S_ref(b._M_elems, __n));
        }
        return r;
    }

    /**
     * @brief min is the applicative version of building the minimum.
     * @param a - first index
     * @param b - second index
     * @return the minimum
     */
    static inline Index min(const Index &a,
                            const Index &b)
    {
        Index r;
        for(std::size_t __n = 0 ; __n < _Nm ; ++__n) {
            Base::_AT_Type::_S_ref(r._M_elems, __n) =
                    std::min(Base::_AT_Type::_S_ref(a._M_elems, __n),
                             Base::_AT_Type::_S_ref(b._M_elems, __n));
        }
        return r;
    }
};
}
}

//// primitive mathematical operators
template<std::size_t _Nm>
inline muse_mcl::math::Index<_Nm> operator - (const muse_mcl::math::Index<_Nm>& __one,
                                               const muse_mcl::math::Index<_Nm>& __two)
{
    muse_mcl::math::Index<_Nm> __res;
    for(std::size_t i = 0 ; i < _Nm; ++i)
        __res[i] = __one[i] - __two[i];

    return __res;
}

template<std::size_t _Nm>
inline muse_mcl::math::Index<_Nm> operator + (const muse_mcl::math::Index<_Nm>& __one,
                                               const muse_mcl::math::Index<_Nm>& __two)
{
    muse_mcl::math::Index<_Nm> __res;
    for(std::size_t i = 0 ; i < _Nm; ++i)
        __res[i] = __one[i] + __two[i];

    return __res;
}

template<std::size_t _Nm>
inline muse_mcl::math::Index<_Nm> operator - (const muse_mcl::math::Index<_Nm>& __idx,
                                               const int __scalar)
{
    muse_mcl::math::Index<_Nm> __res;
    for(std::size_t i = 0 ; i < _Nm; ++i)
        __res[i] = __idx[i] - __scalar;

    return __res;
}

template<std::size_t _Nm>
inline muse_mcl::math::Index<_Nm> operator + (const muse_mcl::math::Index<_Nm>& __idx,
                                               const int __scalar)
{
    muse_mcl::math::Index<_Nm> __res;
    for(std::size_t i = 0 ; i < _Nm; ++i)
        __res[i] = __idx[i] + __scalar;

    return __res;
}

#endif // INDEX_HPP
