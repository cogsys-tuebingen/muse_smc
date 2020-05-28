#ifndef MUSE_SMC_UPDATE_MODEL_HPP
#define MUSE_SMC_UPDATE_MODEL_HPP

#include <memory>

namespace muse_smc {
template <typename Data_T, typename StateSpace_T, typename WeightIterator_T>
class UpdateModel {
 public:
  virtual std::size_t getModelId() const = 0;
  virtual std::string const &getName() const = 0;
  virtual void apply(const std::shared_ptr<Data_T const> &data,
                     const std::shared_ptr<StateSpace_T const> &state_space,
                     WeightIterator_T weights) = 0;
};
}  // namespace muse_smc

#endif  // MUSE_SMC_UPDATE_MODEL_HPP
