#ifndef MUSE_SMC_TRAITS_HPP
#define MUSE_SMC_TRAITS_HPP

namespace muse_smc {
namespace traits {
/**
 * @brief Defines the data type the filter is working with.
 * @tparam SampleType   sample type to set the trait for
 */
template<typename SampleType>
struct Data {
    static_assert(false, "Data type trait not defined!");
};


/**
 * @brief Defines the data provider type the filter is working with.
 * @tparam SampleType   sample type to set the trait for
 */
template<typename SampleType>
struct DataProvider {
    static_assert(false, "Data type trait not defined!");
};

/**
 * @brief Defines the state type the filter is working with.
 * @tparam SampleType   sample type to set the trait for
 */
template<typename SampleType>
struct State {
    static_assert(false, "State type trait not defined!");
};

/**
 * @brief Defines the transform type the filter is working with.
 * @tparam SampleType   sample type to set the trait for
 */
template<typename SampleType>
struct Transform {
    static_assert(false, "Transform type trait not defined!");
};

/**
 * @brief Defines the covariance type the filter is working with.
 * @tparam SampleType   sample type to set the trait for
 */
template<typename SampleType>
struct Covariance {
    static_assert(false, "Covariance type trait not defined!");
};

/**
 * @brief Defines the state space bounary type the filter is working with.
 * @tparam SampleType   sample type to set the trait for
 */
template<typename SampleType>
struct StateSpaceBoundary {
    static_assert(false, "StateSpaceBoundary type trait not defined!");
};
}
}

#endif // MUSE_SMC_TRAITS_HPP
