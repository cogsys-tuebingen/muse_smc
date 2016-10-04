#include <muse_amcl/functions/update_function_factory.h>

using namespace muse_amcl;

UpdateFunctionFactory::UpdateFunctionFactory()
    : plugin_manager_("muse_amcl::Update")
{
    plugin_manager_.load();
}

Update::Ptr UpdateFunctionFactory::create(const std::string& name)
{
    auto constructor = plugin_manager_.getConstructor(name);

    if(constructor) {
        return constructor();

    } else {
        std::cerr << "cannot create update: " << name << std::endl;
        return nullptr;
    }
}
