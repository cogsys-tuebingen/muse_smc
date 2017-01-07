#ifndef PARTICLE_SET_HPP
#define PARTICLE_SET_HPP

#include "particle.hpp"

#include <assert.h>
#include <memory>
#include <vector>
#include <limits>

namespace muse_amcl {
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
            return ParticleMemberIterator<T, Member>(&set.samples_.front());
        }

        ParticleMemberIterator<T, Member> end() {
            return ParticleMemberIterator<T, Member>(&set.samples_.back());
        }
    private:
        ParticleSet& set;
    };

    using PoseIterator = ParticleDecorator<Particle::PoseType, &Particle::pose_>;
    using WeightIterator = ParticleDecorator<Particle::WeightType, &Particle::weight_>;
    using Particles = std::vector<Particle>;

    ParticleSet(const std::string &frame,
                const std::size_t size) :
        frame_(frame),
        max_weight_(0.0),
        samples_(size),
        minimum_size_(size),
        maximum_size_(size)
    {

    }

    ParticleSet(const std::string &frame,
                const std::size_t size,
                const std::size_t minimum_size,
                const std::size_t maximum_size) :
        frame_(frame),
        max_weight_(0.0),
        samples_(size),
        minimum_size_(minimum_size),
        maximum_size_(maximum_size)
    {
    }

    PoseIterator getPoses()
    {
        return PoseIterator(*this);
    }

    WeightIterator getWeights()
    {
        return WeightIterator(*this);
    }

    Particles & getParticles()
    {
        return samples_;
    }

    std::string getFrame() const
    {
        return frame_;
    }


    void resize(const std::size_t sample_size)
    {
        minimum_size_ = sample_size;
        maximum_size_ = sample_size;
        samples_.resize(sample_size);
    }

    void resize(const std::size_t sample_size,
                const std::size_t minimum_size,
                const std::size_t maximum_size)
    {
        samples_.resize(sample_size);
        minimum_size_ = minimum_size;
        maximum_size_ = maximum_size;
    }

    void reserve(const std::size_t sample_size)
    {
        minimum_size_ = sample_size;
        maximum_size_ = sample_size;
        samples_.reserve(sample_size);
    }

    void reserve(const std::size_t sample_size,
                 const std::size_t minimum_size,
                 const std::size_t maximum_size)
    {
        samples_.reserve(sample_size);
        minimum_size_ = minimum_size;
        maximum_size_ = maximum_size;
    }

    void emplace_back(const Particle &sample)
    {
         samples_.emplace_back(sample);
    }

    std::size_t getMinimumSize() const
    {
        return minimum_size_;
    }

    std::size_t getMaximumSize() const
    {
        return maximum_size_;
    }

    std::size_t getSize() const
    {
        return samples_.size();
    }

    double getMaximumWeight() const
    {
        return max_weight_;
    }

    void normalize()
    {
        double W = 0.0;
        for(auto &p : samples_) {
            W += p.weight_;
        }

        max_weight_ = std::numeric_limits<double>::lowest();
        for(auto &p : samples_) {
            p.weight_ /= W;

            if(p.weight_ > max_weight_)
                max_weight_ = p.weight_;
        }
    }

    void normalize(const double W)
    {
        max_weight_ = std::numeric_limits<double>::lowest();
        for(auto &p : samples_) {
            p.weight_ /= W;

            if(p.weight_ > max_weight_)
                max_weight_ = p.weight_;
        }
    }

private:
    std::string frame_;
    double      max_weight_;
    Particles   samples_;
    std::size_t minimum_size_;
    std::size_t maximum_size_;

};
}

#endif /* PARTICLE_SET_HPP */
