#include <muse_amcl/plugins/plugin_factory.hpp>
#include <muse_amcl/data_sources/map_provider.hpp>
#include <muse_amcl/data_sources/data_provider.hpp>

#include <muse_amcl/particle_filter/update.hpp>
#include <muse_amcl/particle_filter/update_manager.hpp>
#include <muse_amcl/particle_filter/propagation.hpp>
#include <muse_amcl/particle_filter/propagation_manager.hpp>
#include <muse_amcl/particle_filter/resampling.hpp>
#include <muse_amcl/particle_filter/pose_generation_uniform.hpp>
#include <muse_amcl/particle_filter/pose_generation_normal.hpp>


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
    muse_amcl::Propagation::Ptr propagation;
    muse_amcl::UniformPoseGeneration::Ptr uniform_pose_generation;
    muse_amcl::NormalPoseGeneration::Ptr  normal_pose_generation;
    muse_amcl::Resampling::Ptr            resampling;
    std::map<std::string, muse_amcl::MapProvider::Ptr> maps;
    std::map<std::string, muse_amcl::DataProvider::Ptr> datas;

    muse_amcl::PluginLoader<muse_amcl::Update>::load(nh, updates);
    propagation = muse_amcl::PluginLoader<muse_amcl::Propagation>::load(nh);
    muse_amcl::PluginLoader<muse_amcl::MapProvider>::load(nh, maps);
    muse_amcl::PluginLoader<muse_amcl::DataProvider>::load(nh, datas);
    uniform_pose_generation = muse_amcl::PluginLoader<muse_amcl::UniformPoseGeneration>::load(nh);
    normal_pose_generation = muse_amcl::PluginLoader<muse_amcl::NormalPoseGeneration>::load(nh);
    resampling = muse_amcl::PluginLoader<muse_amcl::Resampling>::load(nh);

    std::cout << "updates      " << updates.size() << std::endl;
    std::cout << "propagations " << (propagation ? 1 : 0) << std::endl;
    std::cout << "maps         " << maps.size() << std::endl;
    std::cout << "datas        " << datas.size() << std::endl;
    std::cout << "uniform pose " << (uniform_pose_generation ? 1 : 0) << std::endl;
    std::cout << "normal pose  " << (normal_pose_generation ? 1 : 0) << std::endl;
    std::cout << "resampling   " << (resampling ? 1 : 0) << std::endl;

    uniform_pose_generation->setMapProviders(maps);
    normal_pose_generation->setMapProviders(maps);


    muse_amcl::Data::ConstPtr data;
    muse_amcl::Map::ConstPtr map;
    muse_amcl::ParticleSet set(1);
    std::cout << "updates first" << std::endl;
    for(auto &u : updates) {
        u.second->apply(data, map, set.getWeights());

    }
    std::cout << "propagations second" << std::endl;
    propagation->apply(data, set.getPoses());

    muse_amcl::UpdateQueue   uq;
    muse_amcl::UpdateManager um(updates, uq);
    um.bind(datas,
            maps,
            nh);



    muse_amcl::PropagationQueue   pq;
    muse_amcl::PropagationManager pm(propagation, pq);
    pm.bind(datas, nh);

    for(auto &d : datas) {
        d.second->enable();
    }

    while(uq.size() < 10 || pq.size() < 10) {
        std::cout << " ++ " << uq.size() << " " << pq.size() << std::endl;

        ros::spinOnce();
        ros::Rate(3).sleep();
    }

    ros::shutdown();

    return 0;
}