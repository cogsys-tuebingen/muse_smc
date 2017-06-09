#ifndef CLUSTERING_DATA_HPP
#define CLUSTERING_DATA_HPP

#include <muse_mcl/math/weighted_distribution.hpp>
#include <muse_mcl/math/weighted_angular_mean.hpp>
#include <muse_mcl/particle_filter/particle.hpp>
#include <vector>

namespace muse_mcl {
namespace clustering {
/**
 * @brief The Data struct wraps sample information for clustering.
 *        There is a weighted meand and covariance calculation based on the
 *        importance weights of the inserted samples.
 */
struct Data {
    using ParticlePtrs = std::vector<const Particle*>;
    using Distribution = math::statistic::WeightedDistribution<2>;
    using AngularMean  = math::statistic::WeightedAngularMean;

    int              cluster_ = -1;           /// the cluster id
    ParticlePtrs     samples_;                /// samples which belong to the cluster
    Distribution     distribution_;           /// @TODO remove fixed dimension
                                              /// @TODO check what kind of sum
    AngularMean      angular_mean_;           /// angular mean

    /**
     *  @brief Default constructor.
     */
    Data() = default;
    Data(const Data &other) :
        cluster_(other.cluster_),
        samples_((other.samples_)),
        distribution_((other.distribution_)),
        angular_mean_((other.angular_mean_))
    {
    }


    Data(Data &&other) :
        cluster_(other.cluster_),
        samples_(std::move(other.samples_)),
        distribution_(std::move(other.distribution_)),
        angular_mean_(std::move(other.angular_mean_))
    {
    }

    Data& operator = (const Data &other)
    {
        cluster_ = (other.cluster_);
        samples_ = ((other.samples_));
        distribution_ = ((other.distribution_));
        angular_mean_ = ((other.angular_mean_));
        return *this;
    }

    virtual ~Data()
    {
    }


    /**
     * @brief Data constructor.
     * @param sample - sample first introduced into the wrapper
     */
    Data(const Particle &sample)
    {
        samples_.emplace_back(&sample);
        distribution_.add(sample.pose_.getPosition2D(), sample.weight_);
        angular_mean_.add(sample.pose_.yaw(), sample.weight_);
    }

    /**
     * @brief Merge operation for this data container. It fuses the samples associated and the
     *        weighted mean and covariance calculation.
     * @param other - another data container
     */
    inline void merge(const Data &other)
    {
        samples_.insert(samples_.end(), other.samples_.begin(), other.samples_.end());
        distribution_ += other.distribution_;
        angular_mean_ += other.angular_mean_;
    }
};
}
}
#endif // CLUSTERING_DATA_HPP
