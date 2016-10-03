#pragma once

#include "particle.hpp"

#include <memory>
#include <vector>

namespace muse {
template<typename T, T Particle::*Member>
class ParticleMemberIterator : public std::iterator<std::random_access_iterator_tag, T>
{
    Particle *data;

    using parent = std::iterator<std::random_access_iterator_tag, T>;
    using iterator = typename parent::iterator;
    using reference = typename parent::reference;

public:
    explicit ParticleMemberIterator(Particle *_begin) :
        data(_begin)
    {
    }

    iterator& operator++()
    {
        ++data;
        return *this;
    }

    bool operator ==(const ParticleMemberIterator<T, Member> &_other) const
    {
        return data == _other.data;
    }

    bool operator !=(const ParticleMemberIterator<T, Member> &_other) const
    {
        return !(*this == _other);
    }

    reference operator *() const
    {
        return (data->*Member);
    }
};

class ParticleSet
{
public:
    typedef std::shared_ptr<ParticleSet> Ptr;
    typedef std::shared_ptr<ParticleSet const> ConstPtr;

    template<typename T, T Particle::*Member>
    class ParticleDecorator {
    public:
        ParticleDecorator(ParticleSet &_set) :
            set(_set)
        {
        }

        ParticleMemberIterator<T, Member> begin()
        {
            return ParticleMemberIterator<T, Member>(&set.samples.front());
        }

        ParticleMemberIterator<T, Member> end() {
            return ParticleMemberIterator<T, Member>(&set.samples.back());
        }
    private:
        ParticleSet& set;
    };

    using PoseIterator = ParticleDecorator<Particle::PoseType, &Particle::pose>;
    using WeightIterator = ParticleDecorator<Particle::WeightType, &Particle::weight>;
    using Particles = std::vector<Particle>;

    ParticleSet(const std::size_t _sample_size) :
        samples(_sample_size)
    {
    }

    PoseIterator getPoses()
    {
        return PoseIterator(*this);
    }

    WeightIterator getWeigts()
    {
        return WeightIterator(*this);
    }

    Particles & getParticles()
    {
        return samples;
    }

    void resize(const std::size_t _sample_size)
    {
        samples.resize(_sample_size);
    }

    void reserve(const std::size_t _sample_size)
    {
        samples.reserve(_sample_size);
    }

    void emplace_back(const Particle &_sample)
    {
        samples.emplace_back(_sample);
    }

private:
    std::vector<Particle> samples;

};
}
