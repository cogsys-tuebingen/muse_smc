#ifndef NORMAL_SAMPLER_HPP
#define NORMAL_SAMPLER_HPP

#include <memory>
#include <muse_smc/math/random.hpp>
#include "traits.hpp"

namespace muse_smc {
namespace state_space_samplers {
template<typename... Types>
class Normal {
public:
    typedef std::shared_ptr<Normal> Ptr;

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

    inline typename RNG::Vector get()
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

#endif /* NORMAL_SAMPLER_HPP */
