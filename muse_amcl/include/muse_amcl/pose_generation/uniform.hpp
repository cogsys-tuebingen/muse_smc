#pragma once

#include <muse_amcl/math/random.hpp>
#include "arguments.hpp"

namespace muse_amcl {
namespace pose_generation {
template<typename... Types>
class Uniform {
public:
    static_assert(sizeof...(Types) > 0, "Constraint : Dimension > 0");
    static_assert(is_valid_type<Types...>::value, "Parameter list contains forbidden type!");

    static const std::size_t Dimension = sizeof...(Types);

    using RNG = math::random::Uniform<Dimension>;
    using Vector = typename RNG::Vector;

    Uniform() = delete;
    Uniform(const Uniform &other) = delete;

    Uniform(const Vector &min,
            const Vector &max,
            const unsigned int seed = 0) :
        rng_(min, max, seed)
    {
    }

    inline Vector operator() ()
    {
        Vector sample = rng_.get();
        Arguments<Dimension, Vector, Types...>::normalize(sample);
        return sample;
    }

private:
    RNG rng_;
};
}
}
