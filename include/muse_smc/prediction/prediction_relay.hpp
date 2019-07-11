#ifndef PREDICTION_RELAY_HPP
#define PREDICTION_RELAY_HPP

#include <muse_smc/state_space/state_space_provider.hpp>
#include <muse_smc/smc/smc.hpp>

namespace muse_smc {
template<typename sample_t>
class PredictionRelay
{
public:
    using Ptr                     = std::shared_ptr<PredictionRelay>;
    using smc_t                   = SMC<sample_t>;
    using prediction_t            = Prediction<sample_t>;
    using prediction_model_t      = PredictionModel<sample_t>;
    using state_space_provider_t  = StateSpaceProvider<sample_t>;
    using state_space_t           = StateSpace<sample_t>;
    using data_provider_t         = typename muse_smc::traits::DataProvider<sample_t>::type;
    using data_t                  = typename muse_smc::traits::Data<sample_t>::type;


    PredictionRelay(const typename smc_t::Ptr &smc) :
        smc_(smc)
    {
    }

    virtual ~PredictionRelay() = default;

    inline void relay(const typename prediction_model_t::Ptr &p,
                      const typename data_provider_t::Ptr &d)
    {
        auto callback = [this, p](const typename data_t::ConstPtr &data)
        {
            typename prediction_t::Ptr prediction(new prediction_t(data, p));
            smc_->addPrediction(prediction);
        };

        handle_ = d->connect(callback);
    }

    inline void relay(const typename prediction_model_t::Ptr &p,
                      const typename data_provider_t::Ptr &d,
                      const typename state_space_provider_t::Ptr &s)
    {
        auto callback = [this, p, s](const typename data_t::ConstPtr &data)
        {
            typename state_space_t::ConstPtr ss = s->getStateSpace();
            if (ss) {
                typename prediction_t::Ptr prediction(new prediction_t(data, ss, p));
                smc_->addPrediction(prediction);
            } else {
                std::cerr << "[PredictionRelay]: " << s->getName() << " supplied state space which was zero!" << "\n";
                std::cerr << "[PredictionRelay]: Dropped prediction!" << "\n";
            }
        };
        handle_ = d->connect(callback);
    }

private:
    typename smc_t::Ptr                         smc_;
    typename data_provider_t::connection_t::Ptr handle_;
};
}

#endif // PREDICTION_RELAY_HPP
