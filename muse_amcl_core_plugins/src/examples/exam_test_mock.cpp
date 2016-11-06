#include <muse_amcl/plugin_factories/update_function_factory.h>
#include <muse_amcl/plugin_factories/propagation_function_factory.h>
#include <muse_amcl/plugin_factories/data_provider_factory.h>

#include "../mock/mock_data.hpp"

#include <ros/ros.h>


int i = 0;
void doSth(const muse_amcl::Data::ConstPtr &data)
{
    if(data->isType<muse_amcl::MockData>()) {
        const muse_amcl::MockData *m = data->as<muse_amcl::MockData>();
        std::cout << "sth " << i << " : " << m->value << std::endl;
        ++i;
    }
}

void doSthElse(const muse_amcl::Data::ConstPtr &data)
{
    if(data->isType<muse_amcl::MockData>()) {
        const muse_amcl::MockData *m = data->as<muse_amcl::MockData>();
        std::cout << "sth else " << i << " : " << m->value << std::endl;
    }
}


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "muse_mock");

    muse_amcl::UpdateFunctionFactory uf;
    muse_amcl::PropagationFunctionFactory pf;
    muse_amcl::DataProviderFactory df;

    std::shared_ptr<muse_amcl::Update>      u = uf.create("mock_update",
                                                          "muse_amcl::MockUpdate");
    std::shared_ptr<muse_amcl::Propagation> p = pf.create("mock_propagation",
                                                          "muse_amcl::MockPropagation");
    muse_amcl::ParticleSet set(1);

    u->apply(set.getWeights());
    p->apply(set.getPoses());


    std::shared_ptr<muse_amcl::DataProvider> d = df.create("mock_data",
                                                           "muse_amcl::MockDataProvider");

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
