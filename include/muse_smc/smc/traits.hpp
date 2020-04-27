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
    // type of Data has to be defined by using type = ...
};

/**
 * @brief Defines the data provider type the filter is working with.
 * @tparam SampleType   sample type to set the trait for
 */
template<typename SampleType>
struct DataProvider {
    // type of DataProvider has to be defined by using type = ...
};

/**
 * @brief Defines the state type the filter is working with.
 * @tparam SampleType   sample type to set the trait for
 */
template<typename SampleType>
struct State {
    // type of State has to be defined by using type = ...
};

/**
 * @brief Defines the transform type the filter is working with.
 * @tparam SampleType   sample type to set the trait for
 */
template<typename SampleType>
struct Transform {
    // type of Transform has to be defined by using type = ...
};

/**
 * @brief Defines the covariance type the filter is working with.
 * @tparam SampleType   sample type to set the trait for
 */
template<typename SampleType>
struct Covariance {
    // type of Covariance has to be defined by using type = ...
};

/**
 * @brief Defines the state space bounary type the filter is working with.
 * @tparam SampleType   sample type to set the trait for
 */
template<typename SampleType>
struct StateSpaceBoundary {
    // type of StateSpaceBoundary has to be defined by using type = ...
};
}
}

#endif // MUSE_SMC_TRAITS_HPP
