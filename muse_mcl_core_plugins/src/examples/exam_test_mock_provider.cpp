#include <ros/ros.h>

#include <muse_amcl/data_sources/data_provider.hpp>
#include <muse_amcl/data_sources/tf_provider.hpp>
#include <muse_amcl/plugins/plugin_factory.hpp>


using namespace muse_amcl;

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "muse_mock_data_provider");


    TFProvider::Ptr tf(new TFProvider);
    muse_amcl::PluginFactory<muse_amcl::DataProvider, TFProvider::Ptr, ros::NodeHandle&> dpf;
    ros::NodeHandle nh("~");
    DataProvider::Ptr dp = dpf.create("dataprovider", "muse_amcl::MockDataProvider", tf, nh);

    if(dp) {
        std::cout << "Dataprovider created !" << std::endl;
    } else
        return -1;

    return 0;
}
