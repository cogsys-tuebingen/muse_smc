#include <muse_amcl/plugins/plugin_factory.hpp>
#include <muse_amcl/particle_filter/update.hpp>
#include <muse_amcl/particle_filter/propagation.hpp>
#include <muse_amcl/data_sources/map_provider.hpp>
#include <muse_amcl/data_sources/data_provider.hpp>

#include "../mock/mock_update.h"
#include "../mock/mock_propagation.h"
#include "../mock/mock_data.hpp"

#include <muse_amcl/plugins/plugin_loader.hpp>

#include <ros/ros.h>
#include <regex>
#include <boost/regex.hpp>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "muse_amcl_exam_test_mock_launch");
    ros::NodeHandle nh("~");

    /// direct access
    std::string update_class;
    std::string update_class_base;
    nh.getParam("update0/class", update_class);
    nh.getParam("update0/base_class", update_class_base);


    std::string propagation_class;
    std::string propagation_class_base;
    nh.getParam("propagation0/class", propagation_class);
    nh.getParam("propagation0/base_class", propagation_class_base);

    std::cout << "update " << std::endl;
    std::cout << update_class_base << " :: " << update_class << std::endl;

    std::cout << "propagation " << std::endl;
    std::cout << propagation_class_base << " :: " << propagation_class << std::endl;



    /// iteration

    std::map<std::string, muse_amcl::Update::Ptr> updates;
    std::map<std::string, muse_amcl::Propagation::Ptr> propagations;
    std::map<std::string, muse_amcl::MapProvider::Ptr> maps;
    std::map<std::string, muse_amcl::DataProvider::Ptr> datas;

    muse_amcl::PluginLoader<muse_amcl::Update>::load(nh, updates);
    muse_amcl::PluginLoader<muse_amcl::Propagation>::load(nh, propagations);
    muse_amcl::PluginLoader<muse_amcl::MapProvider>::load(nh, maps);
    muse_amcl::PluginLoader<muse_amcl::DataProvider>::load(nh, datas);

    std::cout << "updates      " << updates.size() << std::endl;
    std::cout << "propagations " << propagations.size() << std::endl;
    std::cout << "maps         " << maps.size() << std::endl;
    std::cout << "datas        " << datas.size() << std::endl;

    muse_amcl::Data::ConstPtr data;
    muse_amcl::Map::ConstPtr map;
    muse_amcl::ParticleSet set(1);
    std::cout << "updates first" << std::endl;
    for(auto &u : updates) {
        u.second->apply(data, map, set.getWeights());

    }
    std::cout << "propagations second" << std::endl;
    for(auto &p : propagations) {
        p.second->apply(data, set.getPoses());
    }

    ros::shutdown();

    return 0;
}
