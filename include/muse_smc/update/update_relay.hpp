#ifndef MUSE_SMC_UPDATE_RELAY_HPP
#define MUSE_SMC_UPDATE_RELAY_HPP

#include <map>
#include <muse_smc/smc/smc.hpp>
#include <muse_smc/state_space/state_space_provider.hpp>

namespace muse_smc {
template <typename UpdateModel_T, typename Update_T, typename DataProvider_T,
          typename Data_T, typename StateSpaceProvider_T>
class UpdateRelay {
 public:
  using Ptr = std::shared_ptr<UpdateRelay>;

  using arguments_t = std::pair<typename DataProvider_T::Ptr,
                                typename StateSpaceProvider_T::Ptr>;
  using map_t = std::map<typename UpdateModel_T::Ptr, arguments_t>;
  using Data_T = typename muse_smc::traits::Data<sample_t>::type;

  inline explicit UpdateRelay(const typename smc_t::Ptr &smc) : smc_{smc} {}

  inline void relay(const map_t &mapping) {
    for (const auto &e : mapping) {
      const auto &u = e.first;
      const auto &d = e.second.first;
      const auto &s = e.second.second;

      /// By design, we do not allow updates to be bound with empty maps.
      auto callback = [this, u, s](const typename Data_T::ConstPtr &data) {
        auto ss = s->getStateSpace();
        if (ss) {
          typename Update_T::Ptr up(new Update_T(data, ss, u));
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
  typename smc_t::Ptr smc_;
  std::vector<typename DataProvider_T::connection_t::Ptr> handles_;
};
}  // namespace muse_smc

#endif  // MUSE_SMC_UPDATE_RELAY_HPP
