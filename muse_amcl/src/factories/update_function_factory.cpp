#include <muse_amcl/plugins/update_function_factory.h>

using namespace muse_amcl;

UpdateFunctionFactory::UpdateFunctionFactory()
{
}

Update::Ptr UpdateFunctionFactory::create(const std::string& class_name)
{
    Update::Ptr update = PluginFactory<Update>::create(class_name);
    if(update) {
        /// setup
    }
    return update;
}
