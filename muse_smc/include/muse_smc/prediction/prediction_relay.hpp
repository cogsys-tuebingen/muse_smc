#ifndef PREDICTION_RELAY_HPP
#define PREDICTION_RELAY_HPP

#include <muse_smc/data/data_provider.hpp>
#include <muse_smc/smc/smc.hpp>

namespace muse_smc {
template<typename sample_t>
class PredictionRelay
{
public:
    using Ptr = std::shared_ptr<PredictionRelay<sample_t>>;
    using smc_t = SMC<sample_t>;
    using data_provider_t = DataProvider<sample_t>;
    using prediction_t = Prediction<sample_t>;
    using prediction_model_t = PredictionModel<sample_t>;
    using data_t = Data;
    using map_t  = std::pair<typename prediction_model_t::Ptr, typename data_provider_t::Ptr>;


    PredictionRelay(const typename smc_t::Ptr &smc) :
        smc_(smc)
    {
    }

    inline void relay(const typename prediction_model_t::Ptr &p,
                      const typename data_provider_t::Ptr &d)
    {
        auto callback = [this, p](const typename data_t::ConstPtr &data)
        {
            typename prediction_t::Ptr prediction(data, p);
            smc_->addPrediction(prediction);
        };

        handle_ = d->connect(callback);
    }

private:
    typename smc_t::Ptr smc_;
    typename data_provider_t::connection_t::Ptr handle_;
};
}

#endif // PREDICTION_RELAY_HPP
