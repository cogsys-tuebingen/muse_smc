#ifndef MUSE_SMC_TRAITS_HPP
#define MUSE_SMC_TRAITS_HPP

#include <type_traits>

namespace muse_smc {
namespace traits {
/**
 * @brief Defines the data type the filter is working with.
 * @tparam Sample_T   sample type to set the trait for
 */
template <typename Sample_T>
struct Data {
  static_assert(!std::is_same<Sample_T, Sample_T>::value,
                "Trait not overriden for sample type. Please define field type.");
};

/**
 * @brief Defines the data provider type the filter is working with.
 * @tparam Sample_T   sample type to set the trait for
 */
template <typename Sample_T>
struct DataProvider {
  static_assert(!std::is_same<Sample_T, Sample_T>::value,
                "Trait not overriden for sample type. Please define field type.");
};

/**
 * @brief Defines the state type the filter is working with.
 * @tparam Sample_T   sample type to set the trait for
 */
template <typename Sample_T>
struct State {
  static_assert(!std::is_same<Sample_T, Sample_T>::value,
                "Trait not overriden for sample type. Please define field type.");
};

/**
 * @brief Defines the transform type the filter is working with.
 * @tparam Sample_T   sample type to set the trait for
 */
template <typename Sample_T>
struct Transform {
  static_assert(!std::is_same<Sample_T, Sample_T>::value,
                "Trait not overriden for sample type. Please define field type.");
};

/**
 * @brief Defines the covariance type the filter is working with.
 * @tparam Sample_T   sample type to set the trait for
 */
template <typename Sample_T>
struct Covariance {
  static_assert(!std::is_same<Sample_T, Sample_T>::value,
                "Trait not overriden for sample type. Please define field type.");
};

/**
 * @brief Defines the state space boundary type the filter is working with.
 * @tparam Sample_T   sample type to set the trait for
 */
template <typename Sample_T>
struct StateSpaceBoundary {
  static_assert(!std::is_same<Sample_T, Sample_T>::value,
                "Trait not overriden for sample type. Please define field type.");
};

template <typename Sample_T>
struct Time {
  static_assert(!std::is_same<Sample_T, Sample_T>::value,
                "Trait not overriden for sample type. Please define field type.");
};
template <typename Sample_T>
struct TimeFrame {
  static_assert(!std::is_same<Sample_T, Sample_T>::value,
                "Trait not overriden for sample type. Please define field type.");
};
template <typename Sample_T>
struct Duration {
  static_assert(!std::is_same<Sample_T, Sample_T>::value,
                "Trait not overriden for sample type. Please define field type.");
};

template <typename Sample_T>
struct Rate {
  static_assert(!std::is_same<Sample_T, Sample_T>::value,
                "Trait not overriden for sample type. Please define field type.");
};
}  // namespace traits
}  // namespace muse_smc

#endif  // MUSE_SMC_TRAITS_HPP
