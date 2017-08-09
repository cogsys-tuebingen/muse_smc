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
    using map_t  = std::pair<prediction_model_t::Ptr, data_provider_t::Ptr>;


    PredictionRelay(const smc_t::Ptr &smc) :
        smc_(smc)
    {
    }

    inline void relay(const prediction_model_t::Ptr &p,
                      const data_provider_t::Ptr &d)
    {
        auto callback = [this, p](const data_t::ConstPtr &data)
        {
            prediction_t::Ptr prediction(data, p);
            smc_->addPrediction(prediction);
        };

        handle_ = d->connect(callback);
    }

private:
    smc_t::Ptr smc_;
    data_provider_t::connection_t::Ptr handle_;
};
}

#endif // PREDICTION_RELAY_HPP
