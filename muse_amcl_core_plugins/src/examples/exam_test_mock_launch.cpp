#include <muse_amcl/plugins/update_function_factory.h>
#include <muse_amcl/plugins/propagation_function_factory.h>

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
    nh.getParam("update/class", update_class);
    nh.getParam("update/base_class", update_class_base);


    std::string propagation_class;
    std::string propagation_class_base;
    nh.getParam("propagation/class", propagation_class);
    nh.getParam("propagation/base_class", propagation_class_base);

    std::cout << "update " << std::endl;
    std::cout << update_class_base << " :: " << update_class << std::endl;

    std::cout << "propagation " << std::endl;
    std::cout << propagation_class_base << " :: " << propagation_class << std::endl;

    /// iteration
    std::cout << "---------------------" << std::endl;

    std::vector<std::string> params;
    nh.getParamNames(params);
    std::string ns = nh.getNamespace();


    boost::regex plugin_class_regex("(" + ns + "/)(.*)(/class)");
    boost::regex plugin_base_class_regex("(" + ns + "/)(.*)(/base_class)");

    struct Plugin {
        std::string class_name;
        std::string base_class_name;
    };

    std::map<std::string, Plugin> plugins;
    boost::cmatch match;
    for(auto &p : params) {
        if(boost::regex_match(p.c_str(), match, plugin_class_regex)) {
            nh.getParam(match[2] + "/class", plugins[match[2]].class_name);
        }
        if(boost::regex_match(p.c_str(), match, plugin_base_class_regex)) {
            nh.getParam(match[2] + "/base_class", plugins[match[2]].base_class_name);
        }

    }

    std::vector<muse_amcl::Update::Ptr> updates;
    std::vector<muse_amcl::Propagation::Ptr> propagations;
    muse_amcl::UpdateFunctionFactory uf;
    muse_amcl::PropagationFunctionFactory pf;


    for(auto &e : plugins) {
        const std::string &name = e.first;
        const std::string &base_class_name = e.second.base_class_name;
        const std::string &class_name = e.second.class_name;

        if(base_class_name == muse_amcl::UpdateFunctionFactory::Type()) {
            muse_amcl::Update::Ptr u = uf.create(name, class_name);
            updates.push_back(u);
        } else if (base_class_name == muse_amcl::PropagationFunctionFactory::Type()) {
            muse_amcl::Propagation::Ptr p = pf.create(name, class_name);
            propagations.push_back(p);
        } else {
            std::cerr << "Cannot determine this base class '" << base_class_name << "'!" << std::endl;
        }
    }

    muse_amcl::ParticleSet set(1);
    std::cout << "updates first" << std::endl;
    for(auto &u : updates) {
        u->apply(set.getWeights());
    }
    std::cout << "propagations second" << std::endl;
    for(auto &p : propagations) {
        p->apply(set.getPoses());
    }


    ros::shutdown();

    return 0;
}
