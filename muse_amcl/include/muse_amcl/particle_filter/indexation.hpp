#ifndef DISCRETE_INDAXATION_HPP
#define DISCRETE_INDAXATION_HPP

#include <memory>
#include <array>

#include "particle.hpp"     /// exchange by path header
#include "../math/index.hpp"    /// exchange by path header

namespace muse_amcl {
class Indexation {
    /// @todo: get to Nd
public:
    using ResolutionType = std::array<double, 3>;
    using IndexType      = math::Index<3>;
    using SizeType       = std::array<std::size_t, 3>;

    /**
     * @brief Indexation default constructor.
     */
    Indexation()
    {
        resolution_.fill(0.0);
    }

    /**
     * @brief Indexation constructor by resolution type.
     * @param resolution - used to descretize given values
     */
    Indexation(const ResolutionType &resolution) :
        resolution_(resolution)
    {
    }

    /**
     * @brief setResolution sets the desired descritization resolution.
     * @param resolution - the preferred resolution
     */
    inline void setResolution(const ResolutionType &resolution)
    {
        assert(resolution[0] > 0);
        assert(resolution[1] > 0);
        assert(resolution[2] > 0);
        resolution_ = resolution;
    }

    /**
     * @brief getResolution returns the current resolution used for discretization.
     * @return - the current resolution
     */
    inline ResolutionType getResolution() const
    {
        return resolution_;
    }

    /**
     * @brief create returns the discrete index for a sample.
     * @param sample - the sample to get the index for
     * @return       - the descrite index of the sample
     */
    inline IndexType create(const Particle &sample) const
    {
        assert(resolution_[0] > 0);
        assert(resolution_[1] > 0);
        assert(resolution_[2] > 0);
        return IndexType({
                             static_cast<int>(std::floor(sample.pose_.x()   / resolution_[0])),
                             static_cast<int>(std::floor(sample.pose_.y()   / resolution_[1])),
                             static_cast<int>(std::floor(sample.pose_.yaw() / resolution_[2])),
                         });
    }

    /**
     * @brief create returns the discrete index for a sample given as an std array.
     * @param sample - the sample to get the index for
     * @return       - the descrite index of the sample
     */
    inline IndexType create(const std::array<double, 3> &sample) const
    {
        assert(resolution_[0] > 0);
        assert(resolution_[1] > 0);
        assert(resolution_[2] > 0);
        return IndexType({
                             static_cast<int>(std::floor(sample[0] / resolution_[0])),
                             static_cast<int>(std::floor(sample[1] / resolution_[1])),
                             static_cast<int>(std::floor(sample[2] / resolution_[2])),
                         });
    }

    /**
     * @brief size returns the size for a descritely allocated datastructure.
     * @param extent - the extent of the desired volume
     * @return       - the descretized size of the volume
     */
    inline SizeType size(const std::array<double, 3> &extent) const
    {
        const  IndexType index = create(extent) + 1;
        return {
            static_cast<std::size_t>(index[0]),
            static_cast<std::size_t>(index[1]),
            static_cast<std::size_t>(index[2]),
        };
    }

    inline SizeType size(const IndexType &min,
                         const IndexType &max)
    {
        const IndexType index = max - min + 1;
        return {
            static_cast<std::size_t>(index[0]),
            static_cast<std::size_t>(index[1]),
            static_cast<std::size_t>(index[2]),
        };
    }
private:
    ResolutionType resolution_;
};
}

#endif // DISCRETE_INDAXATION_HPP