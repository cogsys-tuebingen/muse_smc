#ifndef UPDATE_MODEL_HPP
#define UPDATE_MODEL_HPP

#include <memory>
#include <muse_amcl/data_sources/tf_provider.hpp>
#include <muse_amcl/data_types/data.hpp>
#include <muse_amcl/data_types/map.hpp>
#include <muse_amcl/particle_filter/particle_set.hpp>

namespace muse_amcl {
class UpdateModel {
public:
    using Ptr = std::shared_ptr<UpdateModel>;

    UpdateModel()
    {
    }

    virtual ~UpdateModel()
    {
    }

    inline const static std::string Type()
    {
        return "muse_amcl::UpdateModel";
    }

    inline const std::string& getName() const
    {
        return name_;
    }

    inline void setup(const std::string     &name,
                      const TFProvider::Ptr &tf_provider,
                      ros::NodeHandle       &nh_private)
    {
        name_        = name;
        base_frame_  = nh_private.param("base_frame", std::string("base_link"));
        tf_provider_ = tf_provider;
        doSetup(nh_private);
    }

    virtual void update(const Data::ConstPtr &data,
                        const Map::ConstPtr &map,
                        ParticleSet::Weights set) = 0;

protected:
    std::string         name_;
    std::string         base_frame_;
    TFProvider::Ptr     tf_provider_;

    virtual void doSetup(ros::NodeHandle &nh_private) = 0;

    inline std::string privateParameter(const std::string &name) const
    {
        return name_ + "/" + name;
    }


};
}

#endif // UPDATE_MODEL_HPP
