#ifndef PARTICLE_SET_HPP
#define PARTICLE_SET_HPP

#include "buffered_vector.hpp"

#include "particle.hpp"
#include "indexation.hpp"

#include "iterator.hpp"
#include "member_iterator.hpp"
#include "insertion.hpp"

#include <memory>
#include <string>


namespace muse_amcl {
class ParticleSet {
public:
    using Ptr      = std::shared_ptr<ParticleSet>;
    using ConstPtr = std::shared_ptr<ParticleSet const>;

    using Poses     = MemberDecorator<Particle, Particle::PoseType,   &Particle::pose_,   ParticleSet>;
    using Weights   = MemberDecorator<Particle, Particle::WeightType, &Particle::weight_, ParticleSet>;
    using Particles = std::buffered_vector<Particle>;
    using ParticleInsertion = Insertion<ParticleSet>;

    ParticleSet(const ParticleSet &other) = delete;

    ParticleSet(const std::string frame,
                const std::size_t sample_size,
                const Indexation &indexation) :
        frame_(frame),
        sample_size_minimum_(sample_size),
        sample_size_maximum_(sample_size),
        p_t_1(new Particles(0, sample_size)),
        p_t(new Particles(0, sample_size)),
        indexation_(indexation),
        sample_index_minimum_(std::numeric_limits<int>::max()),
        sample_index_maximum_(std::numeric_limits<int>::min()),
        sample_weight_sum_(0.0),
        sample_weight_maximum_(0.0)
    {
    }

    ParticleSet(const std::string frame,
                const std::size_t sample_size_minimum,
                const std::size_t sample_size_maximum,
                const Indexation &indexation) :
        frame_(frame),
        sample_size_minimum_(sample_size_minimum),
        sample_size_maximum_(sample_size_maximum),
        p_t_1(new Particles(0, sample_size_maximum)),
        p_t(new Particles(0, sample_size_maximum)),
        indexation_(indexation),
        sample_index_minimum_(std::numeric_limits<int>::max()),
        sample_index_maximum_(std::numeric_limits<int>::min()),
        sample_weight_sum_(0.0),
        sample_weight_maximum_(0.0)
    {
    }

    inline Poses getPoses()
    {
        sample_index_minimum_  = std::numeric_limits<int>::max();
        sample_index_maximum_  = std::numeric_limits<int>::min();
        return Poses(*p_t_1, *this, &ParticleSet::updateIndices);
    }

    inline Weights getWeights()
    {
        sample_weight_maximum_ = 0.0;
        sample_weight_sum_     = 0.0;
        return Weights(*p_t_1, *this, &ParticleSet::updateWeight);
    }

    inline Particles const & getSamples() const
    {
        return *p_t_1;
    }

    inline ParticleInsertion getInsertion()
    {
        sample_index_minimum_  = std::numeric_limits<int>::max();
        sample_index_maximum_  = std::numeric_limits<int>::min();
        sample_weight_maximum_ = 0.0;
        sample_weight_sum_     = 0.0;

        p_t->clear();
        return ParticleInsertion(*p_t, *this,
                                 &ParticleSet::updateAll,
                                 &ParticleSet::insertionFinished);
    }

    inline void normalizeWeights()
    {
        for(auto &s : *p_t_1) {
            s.weight_ /= sample_weight_sum_;
        }
        sample_weight_maximum_ /= sample_weight_sum_;
        sample_weight_sum_ = 1.0;

    }


    inline std::size_t getSampleSizeMinimum() const
    {
        return sample_size_minimum_;
    }

    inline std::size_t getSampleSizeMaximum() const
    {
        return sample_size_maximum_;
    }

    inline std::size_t getSampleSize() const
    {
        return p_t_1->size();
    }


    inline Indexation::IndexType getSampleIndexMinimum() const
    {
        return sample_index_minimum_;
    }

    inline Indexation::IndexType getSampleIndexMaximum() const
    {
        return sample_index_maximum_;
    }


    inline double getSampleWeightMaximum() const
    {
        return sample_weight_sum_ ? sample_weight_maximum_ / sample_weight_sum_ : 0.0;
    }

    inline double getSampleWeightSum() const
    {
        return sample_weight_sum_;
    }


private:
    using Index = Indexation::IndexType;

    std::string    frame_;
    std::size_t    sample_size_minimum_;
    std::size_t    sample_size_maximum_;
    Indexation     indexation_;

    Index          sample_index_minimum_;
    Index          sample_index_maximum_;
    double         sample_weight_sum_;
    double         sample_weight_maximum_;

    Particles::Ptr p_t_1;   /// set of the previous time step
    Particles::Ptr p_t;     /// set of the upcoming time step

    /// method to be called by the pose iterator
    inline void updateIndices(const Particle::PoseType &sample_pose)
    {
        const Indexation::IndexType i = indexation_.create(sample_pose);
        sample_index_minimum_.min(i);
        sample_index_maximum_.max(i);
    }

    /// method to be called by the weight iterator
    inline void updateWeight(const Particle::WeightType &sample_weight)
    {
        sample_weight_sum_ += sample_weight;
        if(sample_weight_maximum_ < sample_weight)
            sample_weight_maximum_ = sample_weight;
    }

    /// method to be called of resampling insertion
    inline void updateAll(const Particle &sample)
    {
        updateIndices(sample.pose_);
        updateWeight(sample.weight_);
    }

    inline void insertionFinished()
    {
        std::swap(p_t, p_t_1);
    }

};
}

#endif // PARTICLE_SET_HPP
