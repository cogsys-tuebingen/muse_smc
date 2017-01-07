#ifndef DATA_PROVIDER_HPP
#define DATA_PROVIDER_HPP

#include <memory>
#include <functional>
#include <ros/node_handle.h>

#include <muse_amcl/utils/signals.hpp>
#include <muse_amcl/data_types/data.hpp>

namespace muse_amcl {
class DataProvider {
public:
    typedef std::shared_ptr<DataProvider>              Ptr;
    typedef std::function<void(const Data::ConstPtr&)> Callback;
    typedef Signal<Callback>                           DataSignal;
    typedef DataSignal::Connection                     DataConnection;

    DataProvider()
    {
    }

    virtual ~DataProvider()
    {
    }

    inline const static std::string Type()
    {
        return "muse_amcl::DataProvider";
    }

    inline std::string getName() const
    {
        return name_;
    }

    void setup(const std::string &name,
               ros::NodeHandle   &nh_private)
    {
        name_ = name;
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
    std::string name_;
    DataSignal  data_received_;

    virtual void doSetup(ros::NodeHandle &nh_private) = 0;

    std::string param(const std::string &name)
    {
        return name_ + "/" + name;
    }

};
}

#endif /* DATA_PROVIDER_HPP */
