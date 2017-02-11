#ifndef PARTICLE_SET_HPP
#define PARTICLE_SET_HPP

#include <muse_amcl/utils/buffered_vector.hpp>

#include "particle.hpp"
#include "indexation.hpp"
#include "clustering_data.hpp"
#include "clustering_impl.hpp"

#include "insertion.hpp"
#include "member_iterator.hpp"

#include <memory>
#include <string>

#include <cslibs_indexed_storage/storage.hpp>
#include <cslibs_indexed_storage/backend/kdtree/kdtree_buffered.hpp>
#include <cslibs_indexed_storage/backend/array/array.hpp>

namespace muse_amcl {
class ParticleSet {
public:
    /// typedefs :
    using Ptr      = std::shared_ptr<ParticleSet>;
    using ConstPtr = std::shared_ptr<ParticleSet const>;

    using Index     = Indexation::IndexType;
    using Size      = Indexation::SizeType;
    using Poses     = MemberDecorator<Particle, Particle::PoseType,   &Particle::pose_>;
    using Weights   = MemberDecorator<Particle, Particle::WeightType, &Particle::weight_>;
    using Particles = std::buffered_vector<Particle>;
    using Clusters        = std::unordered_map<int, clustering::Data::ParticlePtrs>;
    using Distributions   = std::unordered_map<int, clustering::Data::Distribution>;
    using KDTreeBuffered  = cis::Storage<clustering::Data, Indexation::IndexType::Base, cis::backend::kdtree::KDTreeBuffered>;
    using Array           = cis::Storage<clustering::Data, Indexation::IndexType::Base, cis::backend::array::Array>;
    using KDClustering    = cis::operations::clustering::Clustering<KDTreeBuffered>;
    using ArrayClustering = cis::operations::clustering::Clustering<Array>;
    using ClusteringImpl  = clustering::ClusteringImpl;

    /// particle sets should not be copyable
    ParticleSet(const ParticleSet &other) = delete;
    ParticleSet& operator = (const ParticleSet &other) = delete;

    /**
     * @brief ParticleSet constructor.
     * @param frame         - the frame the particles are defined in
     * @param sample_size   - the sample size
     * @param indexation    - the discretization helper
     * @param array_extent  - the size of the array
     */
    ParticleSet(const std::string frame,
                const std::size_t sample_size,
                const Indexation &indexation,
                const double array_extent = 5.0) :
        frame_(frame),
        sample_size_minimum_(sample_size),
        sample_size_maximum_(sample_size),
        indexation_(indexation),
        sample_index_minimum_(std::numeric_limits<int>::max()),
        sample_index_maximum_(std::numeric_limits<int>::min()),
        sample_weight_sum_(0.0),
        sample_weight_maximum_(0.0),
        sample_weight_average_(0.0),
        p_t_1(new Particles(0, sample_size)),
        p_t(new Particles(0, sample_size)),
        kdtree_(new KDTreeBuffered),
        array_(new Array),
        array_to_be_used_(false)
    {
        kdtree_->set<cis::option::tags::node_allocator_chunk_size>(2 * sample_size_maximum_ + 1);
        array_size_ = indexation_.size({array_extent, array_extent, 2 * M_PI});
        array_->set<cis::option::tags::array_size>(array_size_[0], array_size_[1], array_size_[2]);
    }

    /**
     * @brief ParticleSet constructor.
     * @param frame                 - the frame the particles are defined in
     * @param sample_size_minimum   - the minimum sample set size
     * @param sample_size_maximum   - the maximum sample set size
     * @param indexation            - the discretization helper
     * @param array_extent          - the array size
     */
    ParticleSet(const std::string frame,
                const std::size_t sample_size_minimum,
                const std::size_t sample_size_maximum,
                const Indexation &indexation,
                const double      array_extent = 5.0) :
        frame_(frame),
        sample_size_minimum_(sample_size_minimum),
        sample_size_maximum_(sample_size_maximum),
        indexation_(indexation),
        sample_index_minimum_(std::numeric_limits<int>::max()),
        sample_index_maximum_(std::numeric_limits<int>::min()),
        sample_weight_sum_(0.0),
        sample_weight_maximum_(0.0),
        p_t_1(new Particles(0, sample_size_maximum)),
        p_t(new Particles(0, sample_size_maximum)),
        kdtree_(new KDTreeBuffered),
        array_(new Array),
        array_to_be_used_(false)
    {
        kdtree_->set<cis::option::tags::node_allocator_chunk_size>(2 * sample_size_maximum_ + 1);
        array_size_ = indexation_.size({array_extent, array_extent, 2 * M_PI});
        array_->set<cis::option::tags::array_size>(array_size_[0], array_size_[1], array_size_[2]);
    }

    /**
     * @brief   Move assignemtn operator
     * @param   other - the particle set to be moved
     */
    ParticleSet& operator = (ParticleSet &&other) = default;


    /**
     * @brief   Return a pose access handler. Weights can be iterated using the
     *          encapsulated random access iterators.
     * @return  a pose access handler
     */
    inline Poses getPoses()
    {
        return Poses(*p_t_1,
                     Poses::notify_update::from<ParticleSet, &ParticleSet::updateIndices>(this),
                     Poses::notify_touch::from<ParticleSet, &ParticleSet::resetIndices>(this));
    }

    /**
     * @brief   Return a weight access handler. Weights can be iterated using the
     *          encapsulated random access iterators.
     * @return  a weight access handler
     */
    inline Weights getWeights()
    {
        return Weights(*p_t_1,
                       Weights::notify_update::from<ParticleSet, &ParticleSet::updateWeight>(this),
                       Weights::notify_touch::from<ParticleSet, &ParticleSet::resetWeights>(this));
    }

    /**
     * @brief Read access to the currently active samples.
     * @return
     */
    inline Particles const & getSamples() const
    {
        return *p_t_1;
    }

    /**
     * @brief  Returns a insertion handler object, which can be used to fill the particle set or
     *         for resampling.
     * @return insertion handler
     */
    inline Insertion getInsertion()
    {

        if(p_t_1->size() > 0) {
            /// this means that bounaries have to be update for time step t-1
            const Size size = indexation_.size(sample_index_minimum_, sample_index_maximum_);
            array_to_be_used_ = size[0] <= array_size_[0] && size[1] <= array_size_[1] && size[2] <= array_size_[2];
            array_to_be_used_ = false;
            /// maybe the radian part can be exchanged by static value
        }

        sample_index_minimum_  = std::numeric_limits<int>::max();
        sample_index_maximum_  = std::numeric_limits<int>::min();
        sample_weight_maximum_ = 0.0;
        sample_weight_sum_     = 0.0;
        p_t->clear();
        kdtree_->clear();

        if(array_to_be_used_) {
            array_->set<cis::option::tags::array_offset>(sample_index_minimum_[0],
                                                         sample_index_minimum_[1],
                                                         sample_index_minimum_[2]);
            return Insertion(*p_t,
                             Insertion::notify_update::from<ParticleSet, &ParticleSet::updateInsertArray>(this),
                             Insertion::notify_finished::from<ParticleSet, &ParticleSet::insertionFinished>(this));
        } else {
            return Insertion(*p_t,
                             Insertion::notify_update::from<ParticleSet, &ParticleSet::updateInsertKD>(this),
                             Insertion::notify_finished::from<ParticleSet, &ParticleSet::insertionFinished>(this));
        }
    }

    /**
     * @brief Initiate the weight normalization, such that all weights sum up to 1.0.
     */
    inline void normalizeWeights()
    {
        if(p_t_1->size() == 0)
            return;


        for(auto &s : *p_t_1) {
            s.weight_ /= sample_weight_sum_;
        }
        sample_weight_average_  = sample_weight_sum_ / p_t_1->size();
        sample_weight_maximum_ /= sample_weight_sum_;
        sample_weight_sum_ = 1.0;

    }

    /**
     * @brief Cluster the particle set to generate a density estimate.
     */
    inline void cluster()
    {
        /// run clustering
        ClusteringImpl impl;
        if(array_to_be_used_) {
            ArrayClustering clustering(*array_);
            clustering.cluster(impl);
        } else {
            KDClustering clustering(*kdtree_);
            clustering.cluster(impl);
        }
        p_t_1_clusters_ = std::move(impl.clusters_);
        p_t_1_distributions_ = std::move(impl.distributions_);
    }

    /**
     * @brief Return the clusters, after clustering.
     * @return
     */
    inline Clusters const & getClusters() const
    {
        return p_t_1_clusters_;
    }

    inline Distributions const & getClusterDistributions() const
    {
        return p_t_1_distributions_;
    }

    /**
     * @brief Return the current histogram size, the number of active bins.
     * @return bins used
     */
    inline std::size_t getCurrentHistogramSize() const
    {
        if(array_to_be_used_) {
            return array_->size();
        } else {
            return kdtree_->size();
        }
    }

    /**
     * @brief Return the minimum sample size of the particle set.
     * @return the minimum sample size
     */
    inline std::size_t getSampleSizeMinimum() const
    {
        return sample_size_minimum_;
    }

    /**
     * @brief Return the maximum sample size of the particle set.
     * @return the maximum sample size
     */
    inline std::size_t getSampleSizeMaximum() const
    {
        return sample_size_maximum_;
    }

    /**
     * @brief Return the current sample size of the particle set.
     * @return the current sample size of the set
     */
    inline std::size_t getSampleSize() const
    {
        return p_t_1->size();
    }

    /**
     * @brief Return the current minimum histogram bin index.
     * @return the current minimum index
     */
    inline Indexation::IndexType getSampleIndexMinimum() const
    {
        return sample_index_minimum_;
    }

    /**
     * @brief Return the current maximum histogram bin index.
     * @return the current maximum index
     */
    inline Indexation::IndexType getSampleIndexMaximum() const
    {
        return sample_index_maximum_;
    }

    /**
     * @brief Return the maximum sample weight.
     * @return  maximum sample weight, else 0.0 for uninitialized sets
     */
    inline double getSampleWeightMaximum() const
    {
        return sample_weight_sum_ ? sample_weight_maximum_ / sample_weight_sum_ : 0.0;
    }

    /**
     * @brief Return the current sum of weights.
     * @return  weight sum, 1.0 if particle set is normalized
     */
    inline double getSampleWeightSum() const
    {
        return sample_weight_sum_;
    }

    /**
     * @brief Return if the particle weights are currently normalized.
     * @return
     */
    inline bool isNormalized() const
    {
        return sample_weight_sum_ == 1.0;
    }

    /**
     * @brief Return the frame the particle set is defined in.
     * @return
     */
    inline std::string getFrame() const
    {
        return frame_;
    }

    /**
     * @brief Return the non-normalized weight average for recovery in the resampling step.
     * @return the non-normalized weight average
     */
    inline double getWeightAverage() const
    {
        return sample_weight_average_;
    }
private:
    std::string    frame_;                              /// the frame particles are defined in
    std::size_t    sample_size_minimum_;                /// minimum sample size of the particle set
    std::size_t    sample_size_maximum_;                /// maximum sample size of the particle set
    Indexation     indexation_;                         /// the discritezation

    Index          sample_index_minimum_;               /// current minimum histogram bin index
    Index          sample_index_maximum_;               /// current maximum histogram bin index
    double         sample_weight_sum_;                  /// current weight sum
    double         sample_weight_maximum_;              /// current sample weight maximum
    double         sample_weight_average_;              /// current non-normalized sample weight average

    Particles::Ptr p_t_1;                               /// set of the previous time step
    Particles::Ptr p_t;                                 /// set of the upcoming time step

    std::shared_ptr<KDTreeBuffered> kdtree_;            /// kdtree for wide range density estimation
    std::shared_ptr<Array>          array_;             /// array for near range density estimation
    bool                            array_to_be_used_;  /// determination is array can used for clustering
    Size                            array_size_;        /// the arrays size

    Clusters                        p_t_1_clusters_;
    Distributions                   p_t_1_distributions_;

    inline void resetWeights()
    {
        sample_weight_maximum_ = 0.0;
        sample_weight_sum_     = 0.0;
    }

    inline void resetIndices()
    {
        sample_index_minimum_  = std::numeric_limits<int>::max();
        sample_index_maximum_  = std::numeric_limits<int>::min();
    }

    /**
     * @brief Histogram index boundaries update callback for pose iterator access.
     * @param sample_pose - the processed sample pose
     */
    inline void updateIndices(const Particle::PoseType &sample_pose)
    {
        const Indexation::IndexType i = indexation_.create(sample_pose);
        sample_index_minimum_.min(i);
        sample_index_maximum_.max(i);
    }

    /**
     * @brief Particle weigh tupdate callback for weight iterator access.
     * @param sample_weight - the weight of the sample processed
     */
    inline void updateWeight(const Particle::WeightType &sample_weight)
    {
        sample_weight_sum_ += sample_weight;
        if(sample_weight_maximum_ < sample_weight)
            sample_weight_maximum_ = sample_weight;
    }

    /**
     * @brief Inseration callback access for the kd-tree implementation assuming the array cannot be used.
     * @param sample - the current sample processed
     */
    inline void updateInsertKD(const Particle &sample)
    {
        /// weight reset to one ;) - more stable and suggested by the "probabilistic robotics"
        kdtree_->insert(indexation_.create(sample), clustering::Data(sample));
        updateIndices(sample.pose_);
        updateWeight(sample.weight_);
    }

    /**
     * @brief Inseration callback access for the array implementation assuming the array can be used.
     * @param sample - the current sample processed
     */
    inline void updateInsertArray(const Particle &sample)
    {
        /// weight reset to one ;) - more stable and suggested by the "probabilistic robotics"
        array_->insert(indexation_.create(sample), clustering::Data(sample));
        updateIndices(sample.pose_);
        updateWeight(sample.weight_);
    }
    /**
     * @brief Finalizing the insertion, the particle storages have to be swapped.
     */
    inline void insertionFinished()
    {
        p_t_1_clusters_.clear();
        std::swap(p_t, p_t_1);

        normalizeWeights();
    }
};
}

#endif // PARTICLE_SET_HPP
