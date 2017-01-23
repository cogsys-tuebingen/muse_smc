#ifndef PARTICLE_SET_HPP
#define PARTICLE_SET_HPP

#include "buffered_vector.hpp"

#include "particle.hpp"
#include "indexation.hpp"
#include "clustering_data.hpp"
#include "clustering.hpp"

#include "iterator.hpp"
#include "member_iterator.hpp"
#include "insertion.hpp"

#include <memory>
#include <string>

#include <cslibs_indexed_storage/storage.hpp>
#include <cslibs_indexed_storage/backend/kdtree/kdtree_buffered.hpp>
#include <cslibs_indexed_storage/backend/array/array.hpp>

namespace muse_amcl {
class ParticleSet {
public:
    using Ptr      = std::shared_ptr<ParticleSet>;
    using ConstPtr = std::shared_ptr<ParticleSet const>;

    using Index     = Indexation::IndexType;
    using Size      = Indexation::SizeType;
    using Poses     = MemberDecorator<Particle, Particle::PoseType,   &Particle::pose_,   ParticleSet>;
    using Weights   = MemberDecorator<Particle, Particle::WeightType, &Particle::weight_, ParticleSet>;
    using Particles = std::buffered_vector<Particle>;
    using ParticleInsertion = Insertion<ParticleSet>;
    using KDTreeBuffered = cis::Storage<clustering::Data, Indexation::IndexType::Base, cis::backend::kdtree::KDTreeBuffered>;
    using Array          = cis::Storage<clustering::Data, Indexation::IndexType::Base, cis::backend::array::Array>;

    ParticleSet(const ParticleSet &other) = delete;

    ParticleSet(const std::string frame,
                const std::size_t sample_size,
                const Indexation &indexation,
                const double array_extent = 5.0) :
        frame_(frame),
        sample_size_minimum_(sample_size),
        sample_size_maximum_(sample_size),
        p_t_1(new Particles(0, sample_size)),
        p_t(new Particles(0, sample_size)),
        indexation_(indexation),
        sample_index_minimum_(std::numeric_limits<int>::max()),
        sample_index_maximum_(std::numeric_limits<int>::min()),
        sample_weight_sum_(0.0),
        sample_weight_maximum_(0.0),
        kdtree_(new KDTreeBuffered),
        array_(new Array),
        array_can_be_used_(false)
    {
        kdtree_->set<cis::option::tags::node_allocator_chunk_size>(2 * sample_size_maximum_ + 1);

        array_size_ = indexation_.size({array_extent, array_extent, 2 * M_PI});
        array_->set<cis::option::tags::array_size>(array_size_[0], array_size_[1], array_size_[2]);
    }

    ParticleSet(const std::string frame,
                const std::size_t sample_size_minimum,
                const std::size_t sample_size_maximum,
                const Indexation &indexation,
                const double      array_extent = 5.0) :
        frame_(frame),
        sample_size_minimum_(sample_size_minimum),
        sample_size_maximum_(sample_size_maximum),
        p_t_1(new Particles(0, sample_size_maximum)),
        p_t(new Particles(0, sample_size_maximum)),
        indexation_(indexation),
        sample_index_minimum_(std::numeric_limits<int>::max()),
        sample_index_maximum_(std::numeric_limits<int>::min()),
        sample_weight_sum_(0.0),
        sample_weight_maximum_(0.0),
        kdtree_(new KDTreeBuffered),
        array_(new Array),
        array_can_be_used_(false)
    {
        kdtree_->set<cis::option::tags::node_allocator_chunk_size>(2 * sample_size_maximum_ + 1);
        array_size_ = indexation_.size({array_extent, array_extent, 2 * M_PI});
        array_->set<cis::option::tags::array_size>(array_size_[0], array_size_[1], array_size_[2]);
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

        if(p_t_1->size() > 0) { /// this means that bounaries have to be update for time step t-1
            const Size size = indexation_.size(sample_index_minimum_, sample_index_maximum_);
            array_can_be_used_ = size[0] <= array_size_[0] && size[1] <= array_size_[1] && size[2] <= array_size_[2];
            /// maybe the radian part can be exchanged by static value
        }

        sample_index_minimum_  = std::numeric_limits<int>::max();
        sample_index_maximum_  = std::numeric_limits<int>::min();
        sample_weight_maximum_ = 0.0;
        sample_weight_sum_     = 0.0;
        p_t->clear();
        kdtree_->clear();

        if(array_can_be_used_) {
            array_->set<cis::option::tags::array_offset>(sample_index_minimum_[0],
                                                         sample_index_minimum_[1],
                                                         sample_index_minimum_[2]);
            return ParticleInsertion(*p_t, *this,
                                     &ParticleSet::updateInsertArray,
                                     &ParticleSet::insertionFinished);
        } else {
            return ParticleInsertion(*p_t, *this,
                                     &ParticleSet::updateInsertKD,
                                     &ParticleSet::insertionFinished);
        }
    }

    inline void normalizeWeights()
    {
        for(auto &s : *p_t_1) {
            s.weight_ /= sample_weight_sum_;
        }
        sample_weight_maximum_ /= sample_weight_sum_;
        sample_weight_sum_ = 1.0;

    }

    inline void updateDensityEstimation()
    {
        /// run clustering
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

    std::shared_ptr<KDTreeBuffered> kdtree_;    /// wide range density estimation
    std::shared_ptr<Array>          array_;     /// near range density estimation
    bool                            array_can_be_used_;
    Size                            array_size_;

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
    inline void updateInsertKD(const Particle &sample)
    {
        kdtree_->insert(indexation_.create(sample), clustering::Data(sample));
        updateIndices(sample.pose_);
        updateWeight(sample.weight_);
    }

    inline void updateInsertArray(const Particle &sample)
    {
        array_->insert(indexation_.create(sample), clustering::Data(sample));
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
