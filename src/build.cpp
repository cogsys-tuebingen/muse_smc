#include <cslibs_time/rate.hpp>
#include <cslibs_time/time.hpp>

#include <muse_smc/smc/traits/sample.hpp>
#include <muse_smc/smc/smc.hpp>


// example namespace containing the required structs / classes / types
namespace example {
struct Sample {};
struct Data {};
struct DataProvider {};
struct State {};
struct Transform {};
struct Covariance {};
struct StateSpaceBoundary {};
}  // namespace example
// Override the traits for a specific example
namespace muse_smc {
namespace traits {
template <>
struct Data<::example::Sample> {
  using type = ::example::Data;
};
template <>
struct DataProvider<::example::Sample> {
  using type = ::example::DataProvider;
};
template <>
struct State<::example::Sample> {
  using type = ::example::State;
};
template <>
struct Transform<::example::Sample> {
  using type = ::example::Transform;
};
template <>
struct Covariance<::example::Sample> {
  using type = ::example::Covariance;
};
template <>
struct StateSpaceBoundary<::example::Sample> {
  using type = ::example::StateSpaceBoundary;
};
template <>
struct Time<::example::Sample> {
  using type = cslibs_time::Time;
};
template <>
struct Duration<::example::Sample> {
  using type = cslibs_time::Duration;
};
template <>
struct Rate<::example::Sample> {
  using type = cslibs_time::Rate;
};
}  // namespace traits
}  // namespace muse_smc

using ExampleSMC = muse_smc::SMC<example::Sample>;

