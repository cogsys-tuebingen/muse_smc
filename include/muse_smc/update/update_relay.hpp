#ifndef MUSE_SMC_UPDATE_RELAY_HPP
#define MUSE_SMC_UPDATE_RELAY_HPP

#include <map>

namespace muse_smc {
template <typename SMC_T, typename UpdateModel_T, typename Update_T, typename DataProvider_T,
          typename Data_T, typename StateSpaceProvider_T>
class UpdateRelay {
 public:
  using arguments_t = std::pair<std::shared_ptr<DataProvider_T>,
                                std::shared_ptr<StateSpaceProvider_T>>;
  using map_t = std::map<std::shared_ptr<UpdateModel_T>, arguments_t>;

  inline explicit UpdateRelay(const std::shared_ptr<SMC_T> &smc) : smc_{smc} {}

  inline void relay(const map_t &mapping) {
    for (const auto &e : mapping) {
      const auto &u = e.first;
      const auto &d = e.second.first;
      const auto &s = e.second.second;

      /// By design, we do not allow updates to be bound with empty maps.
      auto callback = [this, u, s](const std::shared_ptr<Data_T const> &data) {
        auto ss = s->getStateSpace();
        if (ss) {
          std::shared_ptr<Update_T> up(new Update_T(data, ss, u));
          smc_->addUpdate(up);
        } else {
          std::cerr << "[UpdateRelay]: " << s->getName()
                    << " supplied state space which was zero!"
                    << "\n";
          std::cerr << "[UpdateRelay]: Dropped update!"
                    << "\n";
        }
      };
      handles_.emplace_back(d->connect(callback));
    }
  }

 private:
  std::shared_ptr<SMC_T> smc_;
  std::vector<std::shared_ptr<typename DataProvider_T::connection_t>> handles_;
};
}  // namespace muse_smc

#endif  // MUSE_SMC_UPDATE_RELAY_HPP
