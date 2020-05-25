#ifndef MUSE_SMC_UPDATE_MODEL_HPP
#define MUSE_SMC_UPDATE_MODEL_HPP

#include <memory>

namespace muse_smc {
template <typename Data_T, typename StateSpace_T, typename WeightIterator_T>
class UpdateModel {
 public:
  using Ptr = std::shared_ptr<UpdateModel>;
  using ConstPtr = std::shared_ptr<const UpdateModel>;

  virtual std::size_t getModelId() const = 0;
  virtual std::string const &getName() const = 0;
  virtual void apply(const typename Data_T::ConstPtr &data,
                     const typename StateSpace_T::ConstPtr &state_space,
                     WeightIterator_T weights) = 0;
};
}  // namespace muse_smc

#endif  // MUSE_SMC_UPDATE_MODEL_HPP
