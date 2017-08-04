#ifndef DATA_PROVIDER_HPP
#define DATA_PROVIDER_HPP

#include <memory>
#include <functional>

#include <muse/data/data.hpp>
#include <muse/utility/signals.hpp>
#include <muse/utility/delegate.hpp>

namespace muse {
template<typename sample_t>
class DataProvider {
public:
    typedef std::shared_ptr<DataProvider>              Ptr;
    typedef delegate<void(const Data::ConstPtr&)>      callback_t;
    typedef Signal<callback_t>                         signal_t;
    typedef signal_t::Connection                       connection_t;

    DataProvider() = default;
    virtual ~DataProvider() = default;

    inline const static std::string Type()
    {
        return "muse::DataProvider";
    }

    inline std::string getName() const
    {
        return name_;
    }

    void setup(const std::string     &name)
    {
        name_ = name;
    }

    inline std::size_t getId() const
    {
        return id_;
    }

    inline void setId(const std::size_t id)
    {
        id_ = id;
    }

     /**
     * @brief Connect to data provider. Callback will be executed as
     *        long as the connection object is alive.
     * @param callback - function to call
     * @return
     */
    connection_t::Ptr connect(const callback_t &callback)
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
    signal_t          data_received_;
};
}


#endif // DATA_PROVIDER_HPP
