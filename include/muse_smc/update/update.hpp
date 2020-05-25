#ifndef MUSE_SMC_UPDATE_HPP
#define MUSE_SMC_UPDATE_HPP

#include <cslibs_time/time_frame.hpp>

namespace muse_smc {
template <typename update_model_t, typename data_t, typename state_space_t,
          typename weight_iterator_t>
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

  inline explicit Update(const typename data_t::ConstPtr &data,
                         const typename state_space_t::ConstPtr &state_space,
                         const typename update_model_t::Ptr &model)
      : data_{data}, state_space_{state_space}, model_{model} {}

  virtual ~Update() = default;

  inline void operator()(weight_iterator_t weights) {
    model_->update(data_, state_space_, weights);
  }

  inline void apply(weight_iterator_t weights) {
    model_->apply(data_, state_space_, weights);
  }

  inline cslibs_time::Time const &getStamp() const {
    return data_->timeFrame().end;
  }

  inline cslibs_time::Time const &stampReceived() const {
    return data_->stampReceived();
  }

  inline typename update_model_t::Ptr getModel() const { return model_; }

  inline std::string const &getModelName() const { return model_->getName(); }

  inline std::size_t getModelId() const { return model_->getModelId(); }

 private:
  const typename data_t::ConstPtr data_;
  const typename state_space_t::ConstPtr state_space_;
  typename update_model_t::Ptr model_;
};
}  // namespace muse_smc

#endif  // MUSE_SMC_UPDATE_HPP
