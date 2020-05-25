#ifndef MUSE_SMC_UPDATE_HPP
#define MUSE_SMC_UPDATE_HPP

namespace muse_smc {
template <typename UpdateModel_T, typename Data_T, typename StateSpace_T,
          typename WeightIterator_T>
class Update {
 public:
  using Ptr = std::shared_ptr<Update>;
  using ConstPtr = std::shared_ptr<Update const>;

  struct Less {
    bool operator()(const Update &lhs, const Update &rhs) const {
      return lhs.getStamp() < rhs.getStamp();
    }
    bool operator()(const Update::Ptr &lhs, const Update::Ptr &rhs) const {
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
    bool operator()(const Update::Ptr &lhs, const Update::Ptr &rhs) const {
      const auto &lhs_stamp = lhs->getStamp();
      const auto &rhs_stamp = rhs->getStamp();
      return lhs_stamp == rhs_stamp
                 ? lhs->stampReceived() > rhs->stampReceived()
                 : lhs_stamp > rhs_stamp;
    }
  };

  inline explicit Update(const typename Data_T::ConstPtr &data,
                         const typename StateSpace_T::ConstPtr &state_space,
                         const typename UpdateModel_T::Ptr &model)
      : data_{data}, state_space_{state_space}, model_{model} {}

  virtual ~Update() = default;

  inline void operator()(WeightIterator_T weights) {
    model_->update(data_, state_space_, weights);
  }

  inline void apply(WeightIterator_T weights) {
    model_->apply(data_, state_space_, weights);
  }

  inline auto const &getStamp() const {
    return data_->timeFrame().end;
  }

  inline auto const &stampReceived() const {
    return data_->stampReceived();
  }

  inline typename UpdateModel_T::Ptr getModel() const { return model_; }

  inline std::string const &getModelName() const { return model_->getName(); }

  inline std::size_t getModelId() const { return model_->getModelId(); }

 private:
  const typename Data_T::ConstPtr data_;
  const typename StateSpace_T::ConstPtr state_space_;
  typename UpdateModel_T::Ptr model_;
};
}  // namespace muse_smc

#endif  // MUSE_SMC_UPDATE_HPP
