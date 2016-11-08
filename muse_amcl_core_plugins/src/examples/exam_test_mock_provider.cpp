#include <ros/ros.h>

#include <muse_amcl/plugins/types/data_provider.hpp>
#include <muse_amcl/plugins/factory.hpp>


using namespace muse_amcl;

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "muse_mock_data_provider");

    muse_amcl::PluginFactory<muse_amcl::DataProvider> dpf;
    DataProvider::Ptr dp = dpf.create("dataprovider", "muse_amcl::MockDataProvider");

    if(dp) {
        std::cout << "Dataprovider created !" << std::endl;
    } else
        return -1;

    return 0;
}
