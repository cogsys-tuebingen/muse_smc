#pragma once

#include <memory>
#include <functional>
#include <ros/node_handle.h>

#include <muse_amcl/signals/signals.hpp>
#include "data.hpp"

namespace muse_amcl {
class DataProvider {
public:
    typedef std::shared_ptr<DataProvider>       Ptr;
    typedef std::function<void(Data::ConstPtr)> Callback;
    typedef Signal<Callback>                    DataSignal;
    typedef DataSignal::Connection              DataConnection;

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

    inline std::string name() const
    {
        return name_;
    }

    void setup(const std::string &name,
               ros::NodeHandle   &nh_private)
    {
        name_ = name;
        loadParameters(nh_private);
    }

     /**
     * @brief Connect to data provider. Callback will be executed as
     *        long as the connection object is alive.
     * @param callback - function to call
     * @return
     */
    DataConnection::Ptr connect(const Callback &callback)
    {
        return signal_.connect(callback);
    }

    /**
     * @brief Enable data to be pushed through.
     */
    void enable()
    {
        signal_.enable();
    }

    /**
     * @brief Disable data to be pushed through.
     */
    void disable()
    {
        signal_.disable();
    }

protected:
    std::string name_;
    DataSignal  signal_;

        virtual void loadParameters(ros::NodeHandle &nh_private) = 0;

    std::string param(const std::string &name)
    {
        return name_ + "/" + name;
    }

};
}
