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

        inline iterator& operator++()
        {
            set_.update(data_->*Member);
            ++data_;
            return *this;
        }

        inline bool operator ==(const ParticleMemberIterator<T, Member> &_other) const
        {
            return data_ == _other.data_;
        }

        inline bool operator !=(const ParticleMemberIterator<T, Member> &_other) const
        {
            return !(*this == _other);
        }

        inline reference operator *() const
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

        inline ParticleMemberIterator<T, Member> begin()
        {
            return ParticleMemberIterator<T, Member>(&set_.samples_.front(), set_);
        }

        inline ParticleMemberIterator<T, Member> end() {
            return ParticleMemberIterator<T, Member>(&set_.samples_.back(),  set_);
        }
    private:
        ParticleSet& set_;
    };


    /**
     * @brief The ParticleIterator class grants write access to the particles.
     *        As the pose and the weight iterator, it update minimum, maximum index and
     *        the weight sum.
     */
    class ParticleIterator : public std::iterator<std::random_access_iterator_tag, Particle>
    {
        using parent    = std::iterator<std::random_access_iterator_tag, Particle>;
        using iterator  = typename parent::iterator;
        using reference = typename parent::reference;
        using optional  = typename std::function<void(Particle&)>;

        Particle   *data_;
        ParticleSet &set_;

     public:
        explicit ParticleIterator(Particle *begin,
                                  ParticleSet &set) :
            data_(begin),
            set_(set)
        {
             /// enter the data strcture
        }

        inline iterator& operator++()
        {
            set_.update(*data_);
            ++data_;
            return *this;
        }

        inline bool operator ==(const ParticleIterator &_other) const
        {
            return data_ == _other.data_;
        }

        inline bool operator !=(const ParticleIterator &_other) const
        {
            return !(*this == _other);
        }

        inline reference operator *() const
        {
            return *data_;
        }

        /*
         * Should only be used if bounaries are known up front.
         */
        inline void insertIntoDensityEstimation()
        {
            ////
        }
    };

    class Particles {
    public:
        Particles(ParticleSet &set) :
            set_(set)
        {
        }

        inline ParticleIterator begin()
        {
            return ParticleIterator(&set_.samples_.front(), set_);
        }

        inline ParticleIterator end() {
            return ParticleIterator(&set_.samples_.back(), set_);
        }
    private:
        ParticleSet& set_;
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


    /**
     * @brief  Return the pose iterator with write access.
     *         Using this iterator will update the descritized bounds
     * @return the pose iterator
     */
    Poses getPoses()
    {
        minimum_index_  = std::numeric_limits<int>::max();
        maximum_index_  = std::numeric_limits<int>::min();
        return Poses(*this);
    }

    /**
     * @brief   Return the weight iterator with write access to weights.
     *          Iterating particles with this iterator will automatically update the
     *          sum of weights for normalization.
     * @return  the weight iterator
     */
    Weights getWeights()
    {
        sum_of_weights_ = 0.0;
        return Weights(*this);
    }

    /**
     * @brief   Return the particle iterator with write access to weight and position.
     *          This iterator keeps track of the weight sum and the desretized extent of the
     *          particle set.
     * @return  the particle iterator
     */
    Particles getParticles()
    {
        return Particles(*this);
    }

    /**
     * @brief   Return the const particle iterator will leave all set statistics unaltered.
     * @return  With this iterator it is simply possible to check the current state of the set.
     */
    std::vector<Particle> const & getConstParticles()
    {
        return samples_;
    }
    /**
     * @brief  Return the frame name in which the particle set is defined.
     * @return  the frame name
     */
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
        samples_.resize(size);
        minimum_size_ = minimum_size;
        maximum_size_ = maximum_size;
    }

    void reserve(const std::size_t sample_size)
    {
        resetParameterTracking();

        minimum_size_ = sample_size;
        maximum_size_ = sample_size;
        samples_.clear();
        samples_.reserve(sample_size);
    }

    void reserve(const std::size_t minimum_size,
                 const std::size_t maximum_size)
    {
        resetParameterTracking();

        minimum_size_ = minimum_size;
        maximum_size_ = maximum_size;

        samples_.clear();
        samples_.reserve(maximum_size);
    }

    void emplace_back(const Particle &sample)
    {
        update(sample);
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
     *        Maybe pack in to particle weight iterator destructor
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

    void updateDensityEstimation()
    {
        /// clear data storages
        /// loop
        /// and put in samples
    }

private:
    inline void update(const Particle::PoseType &pose)
    {
        const Indexation::IndexType i = indexation_.create(pose);
        minimum_index_.min(i);
        maximum_index_.max(i);
    }

    inline void update(const Particle::WeightType &weight)
    {
        sum_of_weights_ += weight;
    }

    inline void update(const Particle &particle)
    {
        const Indexation::IndexType i = indexation_.create(particle.pose_);
        minimum_index_.min(i);
        maximum_index_.max(i);
        sum_of_weights_ += particle.weight_;
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
