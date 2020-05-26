#ifndef MUSE_SMC_TRAITS_HPP
#define MUSE_SMC_TRAITS_HPP

namespace muse_smc {
namespace traits {
/**
 * @brief Defines the data type the filter is working with.
 * @tparam Sample_T   sample type to set the trait for
 */
template <typename Sample_T>
struct Data {
  // type of Data has to be defined by using type = ...
};

/**
 * @brief Defines the data provider type the filter is working with.
 * @tparam Sample_T   sample type to set the trait for
 */
template <typename Sample_T>
struct DataProvider {
  // type of DataProvider has to be defined by using type = ...
};

/**
 * @brief Defines the state type the filter is working with.
 * @tparam Sample_T   sample type to set the trait for
 */
template <typename Sample_T>
struct State {
  // type of State has to be defined by using type = ...
};

/**
 * @brief Defines the transform type the filter is working with.
 * @tparam Sample_T   sample type to set the trait for
 */
template <typename Sample_T>
struct Transform {
  // type of Transform has to be defined by using type = ...
};

/**
 * @brief Defines the covariance type the filter is working with.
 * @tparam Sample_T   sample type to set the trait for
 */
template <typename Sample_T>
struct Covariance {
  // type of Covariance has to be defined by using type = ...
};

/**
 * @brief Defines the state space boundary type the filter is working with.
 * @tparam Sample_T   sample type to set the trait for
 */
template <typename Sample_T>
struct StateSpaceBoundary {
  // type of StateSpaceBoundary has to be defined by using type = ...
};

template <typename Sample_T>
struct Time {
  // type
};
template <typename Sample_T>
struct TimeFrame {
  // type
};
template <typename Sample_T>
struct Duration {
  // type
};

template<typename Rate_T>
struct Rate {
    // type
};
}  // namespace traits
}  // namespace muse_smc

#endif  // MUSE_SMC_TRAITS_HPP
