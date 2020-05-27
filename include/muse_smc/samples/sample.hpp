#ifndef MUSE_SMC_SAMPLE_HPP
#define MUSE_SMC_SAMPLE_HPP

#include <eigen3/Eigen/Core>
#include <limits>
#include <string>

namespace muse_smc {
template <typename Hypothesis_T, typename State_T, typename Accessor_T, typename Weight_T,
          typename Time_T>
class EIGEN_ALIGN16 Sample {
 public:
  inline Sample() = default;
  inline Sample(const Hypothesis_T &hypothesis, const Weight_T weight)
      : hypothesis_{hypothesis}, weight_{weight} {}

  inline Weight_T & weight() {return weight_;}
  inline Weight_T const & weight() const {return weight_;}
  inline State_T & state() {return Accessor_T::get(hypothesis_);}
  inline State_T const & state() const {return Accessor_T::get(hypothesis_);}

 private:
  Hypothesis_T hypothesis_;
  Weight_T weight_;
};
}  // namespace muse_smc

#endif  // MUSE_SMC_SAMPLE_HPP
