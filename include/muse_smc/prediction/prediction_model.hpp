#ifndef MUSE_SMC_PREDICTION_MODEL_HPP
#define MUSE_SMC_PREDICTION_MODEL_HPP

#include <memory>

namespace muse_smc {
template <typename Data_T, typename StateSpace_T, typename StateIterator_T,
          typename Time_T>
class PredictionModel {
 public:
  struct Result {
    inline Result() = default;
    virtual ~Result() = default;

    inline explicit Result(const std::shared_ptr<Data_T const> &applied)
        : applied{applied} {}

    inline explicit Result(const std::shared_ptr<Data_T const> &applied,
                           const std::shared_ptr<Data_T const> &left_to_apply)
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

    const std::shared_ptr<Data_T const> applied;
    const std::shared_ptr<Data_T const> left_to_apply;
  };

  inline PredictionModel() = default;
  virtual ~PredictionModel() = default;

  virtual std::shared_ptr<Result> apply(const std::shared_ptr<Data_T const> &data,
                                     const Time_T &until,
                                     StateIterator_T states) = 0;

  virtual std::shared_ptr<Result> apply(
      const std::shared_ptr<Data_T const> &data,
      const std::shared_ptr<StateSpace_T const> &state_space, const Time_T &until,
      StateIterator_T states) {
    return apply(data, until, states);
  }
};
}  // namespace muse_smc

#endif  // MUSE_SMC_PREDICTION_MODEL_HPP
