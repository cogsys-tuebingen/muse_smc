#ifndef PREDICTION_MODEL_HPP
#define PREDICTION_MODEL_HPP

#include <muse_amcl/data_sources/tf_provider.hpp>
#include <muse_amcl/data_types/data.hpp>
#include <muse_amcl/particle_filter/particle_set.hpp>

#include <memory>

namespace muse_amcl {
class PredictionModel {
public:
    using Ptr = std::shared_ptr<PredictionModel>;

    struct Result {
        const double         linear_distance_moved;
        const double         angular_distance_moved;
        const Data::ConstPtr left_over;  /// leftover for prediction, either it could not be
                                         /// predicted until time or there is just a part still
                                         /// open to be used for prediction
        Result(const double linear_distance_moved,
               const double angular_distance_moved,
               const Data::ConstPtr &left_over) :
            linear_distance_moved(linear_distance_moved),
            angular_distance_moved(angular_distance_moved),
            left_over(left_over)
        {
        }

        Result(const double linear_distance_moved,
               const double angular_distance_moved) :
            linear_distance_moved(linear_distance_moved),
            angular_distance_moved(angular_distance_moved)
        {
        }

        Result() :
            linear_distance_moved(0.0),
            angular_distance_moved(0.0)
        {
        }

    };

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

    inline void setup(const std::string         &name,
                      const TFProvider::Ptr     &tf_provider,
                      ros::NodeHandle           &nh_private)
    {
        name_        = name;
        base_frame_  = nh_private.param("base_frame", std::string("base_link"));
        tf_provider_ = tf_provider;
        doSetup(nh_private);
    }


    virtual Result predict(const Data::ConstPtr &data,
                           const ros::Time      &until,
                           ParticleSet::Poses    poses) = 0;

protected:
    std::string         name_;
    std::string         base_frame_;
    TFProvider::Ptr     tf_provider_;

    virtual void doSetup(ros::NodeHandle &nh) = 0;

    inline std::string privateParameter(const std::string &name)
    {
        return name_ + "/" + name;
    }


};
}

#endif // PREDICTION_MODEL_HPP
