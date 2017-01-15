#ifndef TEST_UNIFORM_ALL_MAPS_2D_HPP
#define TEST_UNIFORM_ALL_MAPS_2D_HPP

#include "../src/sampling/uniform_all_maps_2d.h"

namespace muse_amcl {
/**
 * @brief The purpose TestUniformPrimary2D struct is to give constant
 *        access to member fields for testing.
 */
struct TestUniformAllMaps2D : UniformAllMaps2D
{
    int getRandomSeed() const
    {
        return random_seed_;
    }

    std::string getName() const
    {
        return name_;
    }

    std::size_t getSampleSize() const
    {
        return sample_size_;
    }

    ros::Duration getSamplingTimeout() const
    {
        return sampling_timeout_;
    }

    ros::Duration getTFTimeout() const
    {
        return tf_timeout_;
    }

    TFProvider::Ptr getTFProvider() const
    {
        return tf_provider_;
    }

    std::vector<MapProvider::Ptr> getMapProviders() const
    {
        return map_providers_;
    }

};
}

#endif // TEST_UNIFORM_ALL_MAPS_2D_HPP
