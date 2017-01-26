#ifndef PREDICTION_FORWARDER_HPP
#define PREDICTION_FORWARDER_HPP

#include "prediction.hpp"
#include "particle_filter.hpp"

namespace muse_amcl  {
class PredictionForwarder {
public:
    using Ptr = std::shared_ptr<PredictionForwarder>;

    PredictionForwarder(const PredictionModel::Ptr &model,
                        const ParticleFilter::Ptr &filter) :
        model_(model),
        filter_(filter)
    {
    }

    inline void bind(const std::map<std::string, DataProvider::Ptr> &data_providers,
                     ros::NodeHandle &nh_private)
    {
        const std::string provider_param = propagation_function_->getName() + "/data_provider";
        const std::string provider_name = nh_private.param<std::string>(provider_param, "");
        const DataProvider::Ptr &provider = data_providers.at(provider_name);

        auto callback = [this] (const Data::ConstPtr &data) {
            Prediction::Ptr p(new Prediction(data->getStamp(), data, model_));
            filter_->addPrediction(p);
        };
        connection_ = provider->connect(callback);
    }


private:
    PredictionModel::Ptr              model_;
    ParticleFilter::Ptr               filter_;
    DataProvider::DataConnection::Ptr connection_;

};
}



#endif // PREDICTION_FORWARDER_HPP
