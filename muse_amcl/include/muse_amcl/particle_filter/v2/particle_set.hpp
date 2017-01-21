#ifndef PARTICLE_SET_HPP
#define PARTICLE_SET_HPP

#include "buffered_vector.hpp"

#include "../particle.hpp"
#include "../indexation.hpp"

#include <memory>
#include <string>


namespace muse_amcl {
class ParticleSet {
public:
    using Ptr      = std::shared_ptr<ParticleSet>;
    using ConstPtr = std::shared_ptr<ParticleSet const>;



    ParticleSet(const std::string frame,
                const std::size_t sample_size,
                const Indexation &indexation) :
        frame_(frame),
        sample_size_minimum_(sample_size),
        sample_size_maximum_(sample_size),
        p_t_1(sample_size),
        p_t(sample_size),
        indexation_(indexation),
        sample_index_minimum_(std::numeric_limits<int>::max()),
        sample_weight_maximum_(std::numeric_limits<int>::min()),
        sample_weight_sum_(0.0),
        sample_weight_maximum_(0.0)
    {
    }

    ParticleSet(const std::string frame,
                const std::size_t sample_size,
                const std::size_t sample_size_minimum,
                const std::size_t sample_size_maximum,
                const Indexation &indexation) :
        frame_(frame),
        sample_size_minimum_(sample_size_minimum),
        sample_size_maximum_(sample_size_maximum),
        p_t_1(sample_size, sample_size_maximum),
        p_t(sample_size, sample_size_maximum),
        indexation_(indexation),
        sample_index_minimum_(std::numeric_limits<int>::max()),
        sample_weight_maximum_(std::numeric_limits<int>::min()),
        sample_weight_sum_(0.0),
        sample_weight_maximum_(0.0)
    {
    }


    /// define pose iterator
    /// weight iterator
    /// resampling iterator /// inserts into index structure
    /// default set iterator





private:
    using Index = Indexation::IndexType;

    std::string frame_;
    std::size_t sample_size_minimum_;
    std::size_t sample_size_maximum_;
    Indexation  indexation_;

    Index       sample_index_minimum_;
    Index       sample_index_maximum_;
    double      sample_weight_sum_;
    double      sample_weight_maximum_;

    std::buffered_vector<Particle> p_t_1;   /// set of the previous time step
    std::buffered_vector<Particle> p_t;     /// set of the upcoming time step

    /// swap and clear when resmpling

};
}

#endif // PARTICLE_SET_HPP
