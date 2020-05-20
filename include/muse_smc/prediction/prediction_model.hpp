#ifndef PREDICTION_MODEL_HPP
#define PREDICTION_MODEL_HPP

#include <memory>
#include <muse_smc/prediction/prediction.hpp>
#include <muse_smc/samples/sample_set.hpp>
#include <muse_smc/state_space/state_space.hpp>

namespace muse_smc {
template <typename SampleSet_T, typename Data_T, typename StateSpace_T>
class PredictionModel {
 public:
  using Ptr = std::shared_ptr<PredictionModel>;

  using data_t = Data_T;
  using sample_set_t = SampleSet_T;
  using state_space_t = StateSpace_T;

  struct Result {
    using Ptr = std::shared_ptr<Result>;
    using ConstPtr = std::shared_ptr<Result const>;

    Result() = default;
    virtual ~Result() = default;

    Result(const typename data_t::ConstPtr &applied) : applied(applied) {}

    Result(const typename data_t::ConstPtr &applied,
           const typename data_t::ConstPtr &left_to_apply)
        : applied(applied), left_to_apply(left_to_apply) {}

    inline bool success() const { return static_cast<bool>(applied); }

    template <typename T>
    bool isType() const {
      const T *t = dynamic_cast<const T *>(this);
      return t != nullptr;
    }

    template <typename T>
    T const &as() const {
      return dynamic_cast<const T &>(*this);
    }

    const typename data_t::ConstPtr applied;
    const typename data_t::ConstPtr left_to_apply;
  };

  PredictionModel() = default;
  virtual ~PredictionModel() = default;

  virtual typename Result::Ptr apply(
      const typename data_t::ConstPtr &data, const cslibs_time::Time &until,
      typename sample_set_t::state_iterator_t states) = 0;

  virtual typename Result::Ptr apply(
      const typename data_t::ConstPtr &data,
      const typename state_space_t::ConstPtr &state_space,
      const cslibs_time::Time &until,
      typename sample_set_t::state_iterator_t states) {
    return apply(data, until, states);
  }
};
}  // namespace muse_smc

#endif  // PREDICTION_MODEL_HPP
