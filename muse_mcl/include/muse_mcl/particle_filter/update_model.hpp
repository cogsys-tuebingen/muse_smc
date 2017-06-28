#ifndef UPDATE_MODEL_HPP
#define UPDATE_MODEL_HPP

#include <memory>
#include <muse_mcl/data_sources/tf_provider.hpp>
#include <muse_mcl/data_types/data.hpp>
#include <muse_mcl/data_types/map.hpp>
#include <muse_mcl/particle_filter/particle_set.hpp>

namespace muse_mcl {
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
        return "muse_mcl::UpdateModel";
    }

    inline const std::string& getName() const
    {
        return name_;
    }

    inline void setup(const std::string     &name,
                      const TFProvider::Ptr &tf_provider,
                      ros::NodeHandle       &nh_private)
    {

        name_             = name;
        world_frame_      = nh_private.param<std::string>("world_frame", "world");
        robot_base_frame_ = nh_private.param<std::string>("robot_base_frame", "base_link");
        tf_timeout_       = ros::Duration(nh_private.param<double>(privateParameter("tf_timeout"), 0.1));
        tf_provider_      = tf_provider;
        doSetup(nh_private);
    }

    virtual void update(const Data::ConstPtr &data,
                        const Map::ConstPtr &map,
                        ParticleSet::Weights set) = 0;

protected:
    std::string         name_;
    std::string         world_frame_;
    std::string         robot_base_frame_;
    ros::Duration       tf_timeout_;
    TFProvider::Ptr     tf_provider_;

    virtual void doSetup(ros::NodeHandle &nh_private) = 0;

    inline std::string privateParameter(const std::string &name) const
    {
        return name_ + "/" + name;
    }


};
}

#endif // UPDATE_MODEL_HPP
