#ifndef PREDICTION_MODEL_HPP
#define PREDICTION_MODEL_HPP


#include <muse_amcl/particle_filter/particle_set.hpp>
#include <muse_amcl/data_sources/tf_provider.hpp>
#include <muse_amcl/data_types/data.hpp>

#include <memory>

namespace muse_amcl {
class PredictionModel {
public:
    using Ptr = std::shared_ptr<PredictionModel>;

    PredictionModel()
    {
    }

    virtual ~PredictionModel()
    {
    }

    inline static const std::string Type()
    {
        return "muse_amcl::PredictionModel";
    }

    inline const std::string& getName() const
    {
        return name_;
    }

    void setup(const std::string     &name,
               const TFProvider::Ptr &tf_provider,
               ros::NodeHandle       &nh_private)
    {
        name_        = name;
        base_frame_  = nh_private.param("base_frame", std::string("base_link"));
        tf_provider_ = tf_provider;
        doSetup(nh_private);
    }

    virtual void predict(const Data::ConstPtr &data, ParticleSet::Poses set) = 0;

protected:
    std::string name_;
    std::string base_frame_;
    TFProvider::Ptr tf_provider_;

    virtual void doSetup(ros::NodeHandle &nh) = 0;

    inline std::string parameter(const std::string &name)
    {
        return name_ + "/" + name;
    }


};
}

#endif // PREDICTION_MODEL_HPP
