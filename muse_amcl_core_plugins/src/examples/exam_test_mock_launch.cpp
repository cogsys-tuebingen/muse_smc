// #include <muse_amcl/functions/update_function_factory.h>
// #include <muse_amcl/functions/propagation_function_factory.h>

#include <ros/ros.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "muse_amcl_exam_test_mock_launch");
    ros::NodeHandle nh("~");


    std::vector<std::string> plugins;
    if(!nh.getParam("plugins", plugins)) {
        std::cerr << "Need an active plugin list!" << std::endl;
    }

    for(std::string &n : plugins) {
        if(!nh.hasParam(n)) {
            std::cerr << "Could not find entry for '" << n << "'!" << std::endl;
        } else {
            std::cerr << "Have param '"  << n << "'!" << std::endl;
        }
    }




    ros::shutdown();




//    muse_amcl::UpdateFunctionFactory uf;
//    muse_amcl::PropagationFunctionFactory pf;

//    std::shared_ptr<muse_amcl::Update>      u = uf.create("muse_amcl::MockUpdate");
//    std::shared_ptr<muse_amcl::Propagation> p = pf.create("muse_amcl::MockPropagation");

//    muse_amcl::ParticleSet set(1);

//    u->apply(set.getWeights());
//    p->apply(set.getPoses());

    return 0;
}
