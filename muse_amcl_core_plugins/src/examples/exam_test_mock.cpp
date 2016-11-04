#include <muse_amcl/plugin_factories/update_function_factory.h>
#include <muse_amcl/plugin_factories/propagation_function_factory.h>
#include <muse_amcl/plugin_factories/data_provider_factory.h>

#include <ros/ros.h>

void doSth(const muse_amcl::Data::ConstPtr &data)
{

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

    muse_amcl::DataProvider::DataConnection::Ptr c = d->connect(doSth);

    return 0;
}
