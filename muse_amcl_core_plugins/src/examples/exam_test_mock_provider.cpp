#include <ros/ros.h>

#include <muse_amcl/plugin_factories/data_provider_factory.h>
#include <muse_amcl/plugin_factories/map_provider_factory.h>

using namespace muse_amcl;

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "muse_mock_data_provider");

    DataProviderFactory dpf;
    DataProvider::Ptr dp = dpf.create("dataprovider", "muse_amcl::MockDataProvider");

    if(dp) {
        std::cout << "Dataprovider created !" << std::endl;
    } else
        return -1;

    return 0;
}
