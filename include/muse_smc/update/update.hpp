#ifndef MUSE_SMC_UPDATE_HPP
#define MUSE_SMC_UPDATE_HPP

namespace muse_smc {
template <typename UpdateModel_T, typename Data_T, typename StateSpace_T,
          typename WeightIterator_T, typename Time_T>
class Update {
 public:
  struct Less {
    bool operator()(const Update &lhs, const Update &rhs) const {
      return lhs.getStamp() < rhs.getStamp();
    }
    bool operator()(const std::shared_ptr<Update> &lhs, const std::shared_ptr<Update> &rhs) const {
      return lhs->getStamp() < rhs->getStamp();
    }
  };

  struct Greater {
    bool operator()(const Update &lhs, const Update &rhs) const {
      const auto &lhs_stamp = lhs.getStamp();
      const auto &rhs_stamp = rhs.getStamp();
      return lhs_stamp == rhs_stamp ? lhs.stampReceived() > rhs.stampReceived()
                                    : lhs_stamp > rhs_stamp;
    }
    bool operator()(const std::shared_ptr<Update> &lhs, const std::shared_ptr<Update> &rhs) const {
      const auto &lhs_stamp = lhs->getStamp();
      const auto &rhs_stamp = rhs->getStamp();
      return lhs_stamp == rhs_stamp
                 ? lhs->stampReceived() > rhs->stampReceived()
                 : lhs_stamp > rhs_stamp;
    }
  };

  inline explicit Update(const std::shared_ptr<Data_T const> &data,
                         const std::shared_ptr<StateSpace_T const> &state_space,
                         const std::shared_ptr<UpdateModel_T> &model)
      : data_{data}, state_space_{state_space}, model_{model} {}

  virtual ~Update() = default;

  inline void operator()(WeightIterator_T weights) {
    model_->update(data_, state_space_, weights);
  }

  inline void apply(WeightIterator_T weights) {
    model_->apply(data_, state_space_, weights);
  }

  inline Time_T const &getStamp() const {
    return data_->timeFrame().end;
  }

  inline Time_T const &stampReceived() const {
    return data_->stampReceived();
  }

  inline std::shared_ptr<UpdateModel_T> getModel() const { return model_; }

  inline std::string const &getModelName() const { return model_->getName(); }

  inline std::size_t getModelId() const { return model_->getModelId(); }

 private:
  const std::shared_ptr<Data_T const> data_;
  const std::shared_ptr<StateSpace_T const> state_space_;
  std::shared_ptr<UpdateModel_T> model_;
};
}  // namespace muse_smc

#endif  // MUSE_SMC_UPDATE_HPP
