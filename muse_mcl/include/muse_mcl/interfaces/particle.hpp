#ifndef PARTICLE_HPP
#define PARTICLE_HPP

#include <memory>

namespace muse_mcl {
template<typename StateT>
struct Particle {
    using Ptr = std::shared_ptr<Particle<StateT>>;

    StateT  state  = StateT();
    double  weight = 1.0;

    inline Particle() = default;

    inline Particle(const StateT &state,
                    const double weight) :
        state(state),
        weight(weight)
    {
    }

    inline Particle(const Particle &other) :
        state(other.state),
        weight(other.weight)
    {
    }

    inline Particle(Particle &&other) :
        state(std::move(other.state)),
        weight(other.weight)
    {
    }

    Particle& operator = (const Particle &other)
    {
        pose = other.pose;
        weight = other.weight;
        return *this;
    }

    Particle& operator = (Particle &&other)
    {
        pose = std::move(other.pose);
        weight = other.weight;
        return *this;
    }

};
}
#endif // PARTICLE_HPP
