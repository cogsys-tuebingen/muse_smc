#ifndef MUSE_SMC_PREDICTION_MODEL_HPP
#define MUSE_SMC_PREDICTION_MODEL_HPP

#include <memory>

namespace muse_smc {
template <typename Data_T, typename StateSpace_T, typename StateIterator_T, typename Time_T>
class PredictionModel {
 public:
  using Ptr = std::shared_ptr<PredictionModel>;
  using ConstPtr = std::shared_ptr<PredictionModel const>;

      struct Result {
    using Ptr = std::shared_ptr<Result>;
    using ConstPtr = std::shared_ptr<Result const>;

    inline Result() = default;
    virtual ~Result() = default;

    inline explicit Result(const typename Data_T::ConstPtr &applied)
        : applied{applied} {}

    inline explicit Result(const typename Data_T::ConstPtr &applied,
                           const typename Data_T::ConstPtr &left_to_apply)
        : applied{applied}, left_to_apply{left_to_apply} {}

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

    const typename Data_T::ConstPtr applied;
    const typename Data_T::ConstPtr left_to_apply;
  };

  inline PredictionModel() = default;
  virtual ~PredictionModel() = default;

  virtual typename Result::Ptr apply(
      const typename Data_T::ConstPtr &data, const Time_T &until,
      StateIterator_T states) = 0;

  virtual typename Result::Ptr apply(
      const typename Data_T::ConstPtr &data,
      const typename StateSpace_T::ConstPtr &state_space,
      const Time_T &until, StateIterator_T states) {
    return apply(data, until, states);
  }
};
}  // namespace muse_smc

#endif  // MUSE_SMC_PREDICTION_MODEL_HPP
