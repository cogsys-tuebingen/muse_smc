#include <muse_amcl/data_sources/data_provider.hpp>
#include <muse_amcl/data_sources/tf_provider.hpp>
#include <muse_amcl/plugins/plugin_factory.hpp>
#include <muse_amcl/particle_filter/update_forwarder.hpp>
#include <muse_amcl/particle_filter/prediction_forwarder.hpp>
#include <muse_amcl/data_types/map.hpp>

#include "../mock/mock_data.hpp"

#include <ros/ros.h>


int i = 0;
void doSth(const muse_amcl::Data::ConstPtr &data)
{
    if(data->isType<muse_amcl::MockData>()) {
        const muse_amcl::MockData &m = data->as<muse_amcl::MockData>();
        std::cout << "sth " << i << " : " << m.value << std::endl;
        ++i;
    }
}

void doSthElse(const muse_amcl::Data::ConstPtr &data)
{
    if(data->isType<muse_amcl::MockData>()) {
        const muse_amcl::MockData &m = data->as<muse_amcl::MockData>();
        std::cout << "sth else " << i << " : " << m.value << std::endl;
    }
}


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "muse_mock");

    muse_amcl::TFProvider::Ptr tf_provider(new muse_amcl::TFProvider);

    muse_amcl::PluginFactory<muse_amcl::UpdateModel, muse_amcl::TFProvider::Ptr, ros::NodeHandle&> uf;
    muse_amcl::PluginFactory<muse_amcl::PredictionModel, muse_amcl::TFProvider::Ptr, ros::NodeHandle&> pf;
    muse_amcl::PluginFactory<muse_amcl::DataProvider, muse_amcl::TFProvider::Ptr, ros::NodeHandle&> df;
    ros::NodeHandle nh("~");

    muse_amcl::UpdateModel::Ptr     u = uf.create("muse_amcl::MockUpdate",
                                                  "MOU",
                                                  tf_provider,
                                                  nh);
    muse_amcl::PredictionModel::Ptr p = pf.create("muse_amcl::MockPropagation",
                                                  "MOP",
                                                  tf_provider,
                                                  nh);
    muse_amcl::Indexation index({0.1, 0.1, 1./18. * M_PI});
    muse_amcl::ParticleSet set("frame", 1, index);

    muse_amcl::Map::ConstPtr map;
    muse_amcl::Data::ConstPtr data;

    if(!u || !p)
        return -1;


    u->update(data, map, set.getWeights());
    p->predict(data, ros::Time::now(), set.getPoses());


    std::shared_ptr<muse_amcl::DataProvider> d = df.create("muse_amcl::MockDataProvider",
                                                           "MOD",
                                                           tf_provider,
                                                           nh);

    muse_amcl::DataProvider::DataConnection::Ptr c1 = d->connect(doSth);
    muse_amcl::DataProvider::DataConnection::Ptr c2 = d->connect(doSthElse);

    d->enable();

    ros::Rate r(10);
    while(i < 10 && ros::ok()) {
        ros::spinOnce();
        r.sleep();
    }


    return 0;
}
