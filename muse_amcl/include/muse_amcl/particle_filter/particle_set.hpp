#ifndef PARTICLE_SET_HPP
#define PARTICLE_SET_HPP

#include "particle.hpp"
#include "indexation_storage.hpp"
#include "iterator.hpp"

#include <assert.h>
#include <memory>
#include <vector>
#include <limits>
#include <functional>

namespace muse_amcl {


class ParticleSet
{
public:
    /**
     * @brief The ParticleMemberIterator class grants access to either pose or weight of the
     *        particles.
     *        With the related member field, boundaries and sum of weights are updated.
     */
    template<typename T, T Particle::*Member>
    class ParticleMemberIterator : public std::iterator<std::random_access_iterator_tag, T>
    {
        using parent    = std::iterator<std::random_access_iterator_tag, T>;
        using iterator  = typename parent::iterator;
        using reference = typename parent::reference;

        Particle    *data_;
        ParticleSet &set_;

    public:
        explicit ParticleMemberIterator(Particle    *begin,
                                        ParticleSet &set) :
            data_(begin),
            set_(set)
        {
        }

        iterator& operator++()
        {
            set_.update(data_->*Member);
            ++data_;
            return *this;
        }

        bool operator ==(const ParticleMemberIterator<T, Member> &_other) const
        {
            return data_ == _other.data_;
        }

        bool operator !=(const ParticleMemberIterator<T, Member> &_other) const
        {
            return !(*this == _other);
        }

        reference operator *() const
        {
            return (data_->*Member);
        }
    };

    /// particle decorator for iteration
    template<typename T, T Particle::*Member>
    class ParticleMemberDecorator {
    public:
        ParticleMemberDecorator(ParticleSet &set) :
            set_(set)
        {
        }

        ParticleMemberIterator<T, Member> begin()
        {
            return ParticleMemberIterator<T, Member>(&set_.samples_.front(), set_);
        }

        ParticleMemberIterator<T, Member> end() {
            return ParticleMemberIterator<T, Member>(&set_.samples_.back(),  set_);
        }
    private:
        ParticleSet& set_;
    };

    class Particles {
    public:

        using optional  = typename std::function<void(Particle&)>;

        Particles(ParticleSet &set) :
            set_(set)
        {
        }

        ParticleIterator begin()
        {
            return ParticleIterator(&set_.samples_.front());
        }

        ParticleIterator end() {
            return ParticleIterator(&set_.samples_.back());
        }
    private:
        ParticleSet& set_;
        optional     opt_;
    };


    /**
     * @brief The ParticleIterator class grants write access to the particles.
     *        In addition to that, this iterator adds particles to the indexation data
     *        storage.
     */
    class ParticleIterator : public std::iterator<std::random_access_iterator_tag, Particle>
    {
        using parent    = std::iterator<std::random_access_iterator_tag, Particle>;
        using iterator  = typename parent::iterator;
        using reference = typename parent::reference;
        using optional  = typename std::function<void(Particle&)>;

        Particle *data_;

     public:
        explicit ParticleIterator(Particle *begin) :
            data_(begin)
        {
             /// enter the data strcture
        }

        iterator& operator++()
        {
            ++data_;
            return *this;
        }

        bool operator ==(const ParticleIterator &_other) const
        {
            return data_ == _other.data_;
        }

        bool operator !=(const ParticleIterator &_other) const
        {
            return !(*this == _other);
        }

        reference operator *() const
        {
            return *data_;
        }
    };


    /// type defs
    using Ptr                = std::shared_ptr<ParticleSet>;
    using ConstPtr           = std::shared_ptr<ParticleSet const>;
    using Poses              = ParticleMemberDecorator<Particle::PoseType,         &Particle::pose_>;
    using Weights            = ParticleMemberDecorator<Particle::WeightType,       &Particle::weight_>;
    using ConstParticles     = std::vector<Particle> const;

    /**
     * @brief ParticleSet
     * @param frame
     * @param size
     */
    ParticleSet(const std::string &frame,
                const std::size_t  size,
                const Indexation  &indexation) :
        minimum_index_(std::numeric_limits<int>::max()),
        maximum_index_(std::numeric_limits<int>::min()),
        sum_of_weights_(0.0),
        maximum_weight_(0.0),
        indexation_(indexation),
        frame_(frame),
        samples_(size),
        minimum_size_(size),
        maximum_size_(size)
    {
        assert(size > 0);
    }

    ParticleSet(const std::string &frame,
                const std::size_t size,
                const std::size_t minimum_size,
                const std::size_t maximum_size,
                const Indexation  &indexation) :
        minimum_index_(std::numeric_limits<int>::max()),
        maximum_index_(std::numeric_limits<int>::min()),
        sum_of_weights_(0.0),
        maximum_weight_(0.0),
        indexation_(indexation),
        frame_(frame),
        samples_(size),
        minimum_size_(minimum_size),
        maximum_size_(maximum_size)
    {
        assert(size <= maximum_size);
        assert(size >= minimum_size);
        samples_.reserve(maximum_size);
    }


    ParticleSet(const ParticleSet &other,
                const bool deep_copy = false) :
        ParticleSet(other.frame_, other.minimum_size_, other.minimum_size_, other.maximum_size_, other.indexation_)
    {
        if(deep_copy) {
            samples_ = other.samples_;
            minimum_index_ = other.minimum_index_;
            maximum_index_ = other.maximum_index_;
            maximum_weight_ = other.maximum_weight_;
            sum_of_weights_ = other.sum_of_weights_;
        }
    }


    Poses getPoses()
    {
        resetParameterTracking();
        return Poses(*this);
    }

    Weights getWeights()
    {
        resetParameterTracking();
        return Weights(*this);
    }

    Particles getParticles()
    {
        return Particles(*this);
    }

    std::vector<Particle> const & getConstParticles()
    {
        return samples_;
    }

    std::string getFrame() const
    {
        return frame_;
    }

    void clear()
    {
        samples_.clear();
        resetParameterTracking();
    }


    void resize(const std::size_t size)
    {
        resetParameterTracking();

        minimum_size_ = size;
        maximum_size_ = size;
        samples_.resize(size);
    }

    void resize(const std::size_t size,
                const std::size_t minimum_size,
                const std::size_t maximum_size)
    {
        resetParameterTracking();

        assert(size <= maximum_size);
        assert(size >= minimum_size);
        samples_.resize(size);
        minimum_size_ = minimum_size;
        maximum_size_ = maximum_size;
    }

    void reserve(const std::size_t sample_size)
    {
        resetParameterTracking();

        minimum_size_ = sample_size;
        maximum_size_ = sample_size;
        samples_.reserve(sample_size);
    }

    void reserve(const std::size_t minimum_size,
                 const std::size_t maximum_size)
    {
        resetParameterTracking();

        samples_.reserve(maximum_size);
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
        return maximum_weight_;
    }

    double getSumOfWeights() const
    {
        return sum_of_weights_;
    }

    Indexation::IndexType getMinimumIndex() const
    {
        return minimum_index_;
    }

    Indexation::IndexType getMaximumIndex() const
    {
        return maximum_index_;
    }

    Indexation getIndexation() const
    {
        return indexation_;
    }

    /**
     * @brief Normalize the particle weights.
     */
    void normalize()
    {
        if(sum_of_weights_ == 0.0) {
            for(auto &p : samples_) {
                sum_of_weights_ += p.weight_;
            }
        }

        maximum_weight_ = std::numeric_limits<double>::lowest();
        for(auto &p : samples_) {
            p.weight_ /= sum_of_weights_;
            if(p.weight_ > maximum_weight_)
                maximum_weight_ = p.weight_;
        }
    }

private:
    void update(const Particle::PoseType &particle)
    {
        const Indexation::IndexType i = indexation_.create(particle);
        minimum_index_.min(i);
        maximum_index_.max(i);
    }

    void update(const Particle::WeightType &weight)
    {
        sum_of_weights_ += weight;
    }


    Indexation::IndexType minimum_index_;
    Indexation::IndexType maximum_index_;
    double                sum_of_weights_;
    double                maximum_weight_;
    Indexation            indexation_;

    /// discretization index
    /// kdtree
    /// array
    /// min max index
    /// update function



    std::string             frame_;
    std::vector<Particle>   samples_;
    std::size_t             minimum_size_;
    std::size_t             maximum_size_;

    inline void resetParameterTracking()
    {
        minimum_index_  = std::numeric_limits<int>::max();
        maximum_index_  = std::numeric_limits<int>::min();
        sum_of_weights_ = 0.0;
        maximum_weight_ = 0.0;
    }

};
}

#endif /* PARTICLE_SET_HPP */
