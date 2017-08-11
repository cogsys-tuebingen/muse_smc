#ifndef UNIFORM_2D_HPP
#define UNIFORM_2D_HPP

#include <muse_smc/sampling/uniform.hpp>

#include <muse_mcl_2d/samples/sample_2d.hpp>
#include <muse_mcl_2d/map/map_provider_2d.hpp>
#include <muse_mcl_2d/tf/tf_provider.hpp>


namespace muse_mcl_2d {
class UniformSampling2D : public muse_smc::SamplingUniform<Sample2D>
{
public:
    using Ptr = std::shared_ptr<UniformSampling2D>;

    UniformSampling2D() = default;
    virtual ~UniformSampling2D() = default;

    void setup(const std::map<std::string, MapProvider2D::Ptr> &map_providers,
               const TFProvider::Ptr &tf,
               ros::NodeHandle &nh)
    {
        auto param_name = [this](const std::string &name){return name_ + "/" + name;};
        sample_size_ = nh.param(param_name("sample_size"), 500);
        sampling_timeout_ = muse_smc::Duration(nh.param(param_name("sampling_timeout"), 10.0));
        tf_ = tf;

        doSetup(map_providers, nh);
    }

protected:
    std::size_t         sample_size_;
    muse_smc::Duration  sampling_timeout_;
    TFProvider::Ptr     tf_;

    virtual void doSetup(const std::map<std::string, MapProvider2D::Ptr> &map_providers,
                         ros::NodeHandle &nh) = 0 ;


};
}


#endif // UNIFORM_2D_HPP
