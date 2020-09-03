#ifndef MUSE_SMC_TRAITS_HYPOTHESIS_HPP
#define MUSE_SMC_TRAITS_HYPOTHESIS_HPP

#include <type_traits>

namespace muse_smc {
namespace traits {
/**
 * @brief Defines the data type the filter is working with.
 * @tparam Hypothesis_T   sample type to set the trait for
 */
template <typename Hypothesis_T>
struct Data {
  static_assert(!std::is_same<Hypothesis_T, Hypothesis_T>::value,
                "Trait not overriden for sample type. Please define field type.");
};

/**
 * @brief Defines the data provider type the filter is working with.
 * @tparam Hypothesis_T   sample type to set the trait for
 */
template <typename Hypothesis_T>
struct DataProvider {
  static_assert(!std::is_same<Hypothesis_T, Hypothesis_T>::value,
                "Trait not overriden for sample type. Please define field type.");
};

/**
 * @brief Defines the state type the filter is working with.
 * @tparam Hypothesis_T   sample type to set the trait for
 */
template <typename Hypothesis_T>
struct State {
  static_assert(!std::is_same<Hypothesis_T, Hypothesis_T>::value,
                "Trait not overriden for sample type. Please define field type.");
};

template <typename Hypothesis_T>
struct StateAccess {
  static_assert(!std::is_same<Hypothesis_T, Hypothesis_T>::value,
                "Trait not overriden for sample type. Please define field type.");
  // static inline State<Hypothesis_T>::type & get(Hypothesis_T &h);
  // static inline State<Hypothesis_T>::type const & get(const Hypothesis_T &h);
};

/**
 * @brief Defines the transform type the filter is working with.
 * @tparam Hypothesis_T   sample type to set the trait for
 */
template <typename Hypothesis_T>
struct Transform {
  static_assert(!std::is_same<Hypothesis_T, Hypothesis_T>::value,
                "Trait not overriden for sample type. Please define field type.");
};

/**
 * @brief Defines the covariance type the filter is working with.
 * @tparam Hypothesis_T   sample type to set the trait for
 */
template <typename Hypothesis_T>
struct Covariance {
  static_assert(!std::is_same<Hypothesis_T, Hypothesis_T>::value,
                "Trait not overriden for sample type. Please define field type.");
};

/**
 * @brief Defines the state space boundary type the filter is working with.
 * @tparam Hypothesis_T   sample type to set the trait for
 */
template <typename Hypothesis_T>
struct StateSpaceBoundary {
  static_assert(!std::is_same<Hypothesis_T, Hypothesis_T>::value,
                "Trait not overriden for sample type. Please define field type.");
};

template <typename Hypothesis_T>
struct Time {
  static_assert(!std::is_same<Hypothesis_T, Hypothesis_T>::value,
                "Trait not overriden for sample type. Please define field type.");
};
template <typename Hypothesis_T>
struct TimeFrame {
  static_assert(!std::is_same<Hypothesis_T, Hypothesis_T>::value,
                "Trait not overriden for sample type. Please define field type.");
};
template <typename Hypothesis_T>
struct Duration {
  static_assert(!std::is_same<Hypothesis_T, Hypothesis_T>::value,
                "Trait not overriden for sample type. Please define field type.");
};

template <typename Hypothesis_T>
struct Rate {
  static_assert(!std::is_same<Hypothesis_T, Hypothesis_T>::value,
                "Trait not overriden for sample type. Please define field type.");
};

template <typename Hypothesis_T>
struct Weight {
  static_assert(!std::is_same<Hypothesis_T, Hypothesis_T>::value,
                "Trait not overriden for sample type. Please define field type.");
};
}  // namespace traits
}  // namespace muse_smc

#endif  // MUSE_SMC_HYPOTHESIS_TRAITS_HPP
