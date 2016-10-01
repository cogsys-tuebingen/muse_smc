#pragma once

#include "particle.hpp"

#include <memory>
#include <vector>

namespace muse {
template<typename T, T Particle::*Member>
class ParticleIterator : public std::iterator<std::random_access_iterator_tag, T>
{
    Particle *data;

    using parent = std::iterator<std::random_access_iterator_tag, T>;
    using iterator = typename parent::iterator;
    using reference = typename parent::reference;

public:
    explicit ParticleIterator(Particle *_begin) :
        data(_begin)
    {
    }

    iterator& operator++()
    {
        ++data;
        return *this;
    }

    bool operator ==(const ParticleIterator<T, Member> &other) const
    {
        return data == other.data;
    }

    bool operator !=(const ParticleIterator<T, Member> &other) const
    {
        return !(*this == other);
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

private:


};
}
