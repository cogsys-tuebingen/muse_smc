#pragma once

#include <muse_amcl/math/random.hpp>
#include "arguments.hpp"

namespace muse_amcl {
namespace pose_generation {
template<typename... Types>
class Normal {
public:
    static_assert(sizeof...(Types) > 0, "Constraint : Dimension > 0");
    static_assert(is_valid_type<Types...>::value, "Parameter list contains forbidden type!");

    static const std::size_t Dimension = sizeof...(Types);

    using RNG = math::random::Normal<Dimension>;

    Normal() = delete;
    Normal(const Normal &other) = delete;

    Normal(const typename RNG::Vector &pose,
           const typename RNG::Matrix &covariance,
           const unsigned int seed = 0) :
        rng_(pose, covariance, seed)
    {
    }

    inline typename RNG::Vector operator () ()
    {
        typename RNG::Vector sample = rng_.get();
        Arguments<Dimension, typename RNG::Vector, Types...>::normalize(sample);
        return sample;
    }

private:
    RNG rng_;
};
}
}
