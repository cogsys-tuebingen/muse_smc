#include <muse_amcl/plugins/propagation_function_factory.h>

using namespace muse_amcl;

PropagationFunctionFactory::PropagationFunctionFactory()
{
}

Propagation::Ptr PropagationFunctionFactory::create(const std::string& class_name)
{
    Propagation::Ptr update = PluginFactory::create(class_name);
    if(update) {
        /// setup
    }
    return update;
}
