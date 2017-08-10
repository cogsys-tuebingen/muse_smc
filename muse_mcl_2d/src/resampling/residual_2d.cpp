#include <muse_smc/resampling/residual.hpp>
#include <muse_mcl_2d/samples/sample_2d.hpp>

#include <ros/ros.h>

#include <class_loader/class_loader_register_macro.h>

namespace muse_mcl_2d {
using Resampling = muse_smc::Resampling<muse_mcl_2d::Sample2D>;
class Residual : public muse_smc::Residual<muse_mcl_2d::Sample2D>
{
public:
    void setup(const typename sample_uniform_t::Ptr &uniform_pose_sampler,
               ros::NodeHandle &nh)
    {
        auto param_name = [this](const std::string &name){return name_ + "/" + name;};
        muse_smc::Residual<muse_mcl_2d::Sample2D>::setup(uniform_pose_sampler,
                                                         nh.param(param_name("recovery_alpha_fast"), 0.0),
                                                         nh.param(param_name("recovery_alpha_slow"), 0.0));
    }
};
}

CLASS_LOADER_REGISTER_CLASS(muse_mcl_2d::Residual, muse_mcl_2d::Resampling)
