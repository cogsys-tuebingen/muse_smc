#ifndef SENSOR_MODEL_FACTORY_H
#define SENSOR_MODEL_FACTORY_H

#include <muse_amcl/sensor/sensor_model.h>
#include <muse_amcl/plugin/plugin_manager.hpp>

namespace muse_amcl {

class SensorModelFactory
{
public:
    SensorModelFactory();

    std::unique_ptr<SensorModel> create(const std::string& name);

private:
    PluginManager<SensorModel> plugin_manager_;
};

}

#endif // SENSOR_MODEL_FACTORY_H
