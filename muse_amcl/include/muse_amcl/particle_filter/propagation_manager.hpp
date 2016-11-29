#ifndef PROPAGATION_MANAGER_HPP
#define PROPAGATION_MANAGER_HPP

#include <memory>
#include <string>

#include "propagation.hpp"
#include "propagation_lambda.hpp"
#include "propagation_queue.hpp"

#include "../data_sources/data_provider.hpp"

namespace muse_amcl {

class PropagationManager {
public:
    typedef std::shared_ptr<PropagationManager> Ptr;

    PropagationManager(Propagation::Ptr  &propagation_function,
                       PropagationQueue  &propagation_queue) :
        propagation_function_(propagation_function),
        propagation_queue_(propagation_queue)
    {
    }

    void bind(const std::map<std::string, DataProvider::Ptr> &data_providers,
              ros::NodeHandle &nh_private)
    {
        const std::string data_provider_param = propagation_function_->name() + "/data_provider";
        const std::string data_provider_name = nh_private.param<std::string>(data_provider_param, "");

        const DataProvider::Ptr &data_provider = data_providers.at(data_provider_name);

        auto callback = [this] (const Data::ConstPtr &data) {
            auto f = [this, data] (ParticleSet::PoseIterator set) {
                propagation_function_->apply(data, set);
            };
            PropagationLambda p(f, data->stamp());
            propagation_queue_.push(p);
        };
        connection_ = data_provider->connect(callback);
    }


private:
    Propagation::Ptr                               &propagation_function_;
    PropagationQueue                               &propagation_queue_;

    DataProvider::DataConnection::Ptr               connection_;
};
}

#endif /* PROPAGATION_MANAGER_HPP */
