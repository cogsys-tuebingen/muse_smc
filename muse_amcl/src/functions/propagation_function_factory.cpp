#include <muse_amcl/functions/propagation_function_factory.h>

using namespace muse_amcl;

PropagationFunctionFactory::PropagationFunctionFactory()
    : plugin_manager_("muse_amcl::Propagation")
{
    plugin_manager_.load();
}

Propagation::Ptr PropagationFunctionFactory::create(const std::string& name)
{
    auto constructor = plugin_manager_.getConstructor(name);

    if(constructor) {
        return constructor();

    } else {
        std::cerr << "cannot create sensor: " << name << std::endl;
        return nullptr;
    }
}
