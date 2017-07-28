#ifndef DATA_PROVIDER_HPP
#define DATA_PROVIDER_HPP

#include <memory>
#include <functional>
#include <ros/node_handle.h>

#include <muse_mcl/tf/tf_provider.hpp>

#include <muse_mcl/utility/signals.hpp>
#include <muse_mcl/utility/delegate.hpp>
#include <muse_mcl/data/data.hpp>

namespace muse_mcl {
class ProviderData {
public:
    typedef std::shared_ptr<ProviderData>              Ptr;
    typedef delegate<void(const Data::ConstPtr&)>      Callback;
    typedef Signal<Callback>                           DataSignal;
    typedef DataSignal::Connection                     DataConnection;

    ProviderData()
    {
    }

    virtual ~ProviderData()
    {
    }

    inline const static std::string Type()
    {
        return "muse_mcl::ProviderData";
    }

    inline std::string getName() const
    {
        return name_;
    }

    void setup(const std::string     &name,
               const TFProvider::Ptr &tf_provider,
               ros::NodeHandle       &nh_private)
    {
        name_ = name;
        tf_provider_ = tf_provider;
        doSetup(nh_private);
    }

     /**
     * @brief Connect to data provider. Callback will be executed as
     *        long as the connection object is alive.
     * @param callback - function to call
     * @return
     */
    DataConnection::Ptr connect(const Callback &callback)
    {
        return data_received_.connect(callback);
    }

    /**
     * @brief Enable data to be pushed through.
     */
    void enable()
    {
        data_received_.enable();
    }

    /**
     * @brief Disable data to be pushed through.
     */
    void disable()
    {
        data_received_.disable();
    }

protected:
    std::string       name_;
    DataSignal        data_received_;
    TFProvider::Ptr   tf_provider_;

    virtual void doSetup(ros::NodeHandle &nh_private) = 0;

    std::string privateParameter(const std::string &name)
    {
        return name_ + "/" + name;
    }

};
}

#endif /* DATA_PROVIDER_HPP */
