#ifndef UPDATE_RELAY_HPP
#define UPDATE_RELAY_HPP

#include <map>

#include <muse_smc/state_space/state_space_provider.hpp>
#include <muse_smc/update/update.hpp>
#include <muse_smc/smc/smc.hpp>

namespace muse_smc {
template<typename state_space_description_t, typename data_t, typename data_provider_t>
class UpdateRelay
{
public:
    using Ptr                    = std::shared_ptr<UpdateRelay>;
    using sample_t               = typename state_space_description_t::sample_t;
    using smc_t                  = SMC<state_space_description_t, data_t>;
    using update_t               = Update<state_space_description_t, data_t>;
    using update_model_t         = UpdateModel<state_space_description_t, data_t>;
    using state_space_provider_t = StateSpaceProvider<state_space_description_t>;
    using state_space_t          = StateSpace<state_space_description_t>;
    using arguments_t            = std::pair<typename data_provider_t::Ptr,
                                             typename state_space_provider_t::Ptr>;
    using map_t                  = std::map<typename update_model_t::Ptr,
                                            arguments_t>;

    inline UpdateRelay(const typename smc_t::Ptr &smc) :
        smc_(smc)
    {
    }

    inline void relay(const map_t &mapping)
    {
        for(const auto &e : mapping) {
            const auto &u = e.first;
            const auto &d = e.second.first;
            const auto &s = e.second.second;

            /// By design, we do not allow updates to be bound with empty maps.
            auto callback = [this, u, s](const typename data_t::ConstPtr &data) {
                typename state_space_t::ConstPtr ss = s->getStateSpace();
                if(ss) {
                    typename update_t::Ptr up(new update_t(data, ss, u));
                    smc_->addUpdate(up);
                } else {
                    std::cerr << "[UpdateRelay]: " << s->getName() << " supplied state space which was zero!" << "\n";
                    std::cerr << "[UpdateRelay]: Dropped update!" << "\n";
                }
            };
            handles_.emplace_back(d->connect(callback));
        }
    }

private:
    typename smc_t::Ptr                                      smc_;
    std::vector<typename data_provider_t::connection_t::Ptr> handles_;
};
}

#endif // UPDATE_RELAY_HPP
