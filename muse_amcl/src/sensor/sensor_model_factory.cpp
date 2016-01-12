#include <muse_amcl/sensor/sensor_model_factory.h>

using namespace muse_amcl;

SensorModelFactory::SensorModelFactory()
    : plugin_manager_("muse_amcl::SensorModel")
{
    plugin_manager_.load();
}

std::unique_ptr<SensorModel> SensorModelFactory::create(const std::string& name)
{
    auto constructor = plugin_manager_.getConstructor(name);

    if(constructor) {
        return constructor();

    } else {
        std::cerr << "cannot create sensor: " << name << std::endl;
        return nullptr;
    }
}
