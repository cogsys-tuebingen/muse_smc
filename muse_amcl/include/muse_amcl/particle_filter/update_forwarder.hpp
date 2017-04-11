#ifndef UPDATE_FORWARDER_HPP
#define UPDATE_FORWARDER_HPP

#include "update.hpp"
#include "particle_filter.hpp"

#include <muse_amcl/data_sources/map_provider.hpp>
#include <muse_amcl/utils/logger.hpp>

namespace muse_amcl {
class UpdateForwarder {
public:
    using Ptr = std::shared_ptr<UpdateForwarder>;

    UpdateForwarder(const std::map<std::string, UpdateModel::Ptr> &models,
                    const ParticleFilter::Ptr &filter) :
        models_(models),
        filter_(filter)
    {
    }

    inline void bind(const std::map<std::string, DataProvider::Ptr> &data_providers,
                     const std::map<std::string, MapProvider::Ptr> &map_providers,
                     ros::NodeHandle &nh_private)
    {
        for(const auto &u : models_) {
            const std::string &model_name = u.first;
            const std::string data_provider_param    = model_name + "/data_provider";
            const std::string map_provider_param     = model_name + "/map_provider";

            const std::string data_provider_name     = nh_private.param<std::string>(data_provider_param, "");
            const std::string map_provider_name      = nh_private.param<std::string>(map_provider_param, "");

            UpdateModel::Ptr  model = models_.at(model_name);
            const DataProvider::Ptr &data_provider= data_providers.at(data_provider_name);
            const MapProvider::Ptr  &map_provider = map_providers.at(map_provider_name);

            auto callback = [this, model, map_provider] (const Data::ConstPtr &data) {
                Map::ConstPtr map = map_provider->getMap();
                if(map) {
                    Update::Ptr u(new Update(data, map, model));
                    filter_->addUpdate(u);
                 } else {
                    Logger::getLogger().error("Could not get map from '" + map_provider->getName() + "'.", "UpdateForwarder");
                 }
            };

            connections_[model_name] = data_provider->connect(callback);
        }
    }

private:
    const std::map<std::string, UpdateModel::Ptr> &models_;
    ParticleFilter::Ptr                            filter_;

    std::map<std::string, DataProvider::DataConnection::Ptr> connections_;
};
}


#endif // UPDATE_FORWARDER_HPP
