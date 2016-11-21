#pragma once

#include "../math/random.hpp"
#include "../data_types/map.hpp"
#include "../particle_filter/particle_set.hpp"
#include "arguments.hpp"

namespace muse_amcl {
namespace pose_generation {
template<typename... Types>
class Normal {
public:
    static_assert(sizeof...(Types) > 0, "Constraint : Dimension > 0");
    static_assert(is_valid_type<Types...>::value, "Parameter list contains forbidden type!");

    static const std::size_t Dimension = sizeof...(Types);
    static constexpr std::array<int, sizeof...(Types)> types{{Types::info...}};

    using RNG = math::random::Normal<Dimension>;

    Normal(const typename RNG::Vector &pose,
           const typename RNG::Matrix &covariance,
           const unsigned int seed = 0) :
        rng_(pose, covariance, seed)
    {
    }

    inline void operator () (const std::size_t                 sample_count,
                             const std::vector<Map::ConstPtr> &maps,
                             ParticleSet                      &particle_set)
    {
        /// first check if the initial pose is blocked
    }

    inline void operator () (const std::size_t sample_count,
                             ParticleSet &particle_set)
    {
        std::size_t count = 0;
        while(count < sample_count) {

        }
    }

    inline typename RNG::Vector operator () (const std::vector<Map::ConstPtr> &maps)
    {
        /// first check if the mean is blocked
    }

    inline typename RNG::Vector operator () ()
    {

    }

private:
    RNG rng_;
};
}
}

/*
 * Concept : write a template class that accepts dimensions of interest as template
 * parameter -> therefor we can specialize what needs to be scrambled
 * - Remember angular values must always be normalized !!!
 * - It should be correlated
 */
