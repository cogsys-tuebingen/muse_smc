#ifndef NORMAL_2D_HPP
#define NORMAL_2D_HPP

#include <muse_smc/sampling/normal.hpp>

#include <muse_mcl_2d/samples/sample_2d.hpp>
#include <muse_mcl_2d/map/map_provider_2d.hpp>
#include <muse_mcl_2d/map/map_2d.hpp>
#include <muse_mcl_2d/tf/tf_provider.hpp>

#include <ros/time.h>

namespace muse_mcl_2d {
class NormalSampling2D : public muse_smc::NormalSampling<Sample2D>
{
public:
    using Ptr = std::shared_ptr<NormalSampling2D>;

    NormalSampling2D() = default;
    virtual ~NormalSampling2D() = default;

    void setup(const std::map<std::string, MapProvider2D::Ptr> &map_providers,
               const TFProvider::Ptr &tf,
               ros::NodeHandle &nh)
    {
        auto param_name = [this](const std::string &name){return name_ + "/" + name;};
        sample_size_ = nh.param(param_name("sample_size"), 500);
        sampling_timeout_ = ros::Duration(nh.param(param_name("sampling_timeout"), 10.0));
        tf_timeout_ = ros::Duration(nh.param(param_name("tf_timeout"), 0.1));
        tf_ = tf;

        doSetup(map_providers, nh);
    }

protected:
    std::size_t         sample_size_;
    ros::Duration       sampling_timeout_;
    ros::Duration       tf_timeout_;
    TFProvider::Ptr     tf_;

    virtual void doSetup(const std::map<std::string, MapProvider2D::Ptr> &map_providers,
                         ros::NodeHandle &nh) = 0 ;

};
}

#endif // NORMAL_2D_HPP
