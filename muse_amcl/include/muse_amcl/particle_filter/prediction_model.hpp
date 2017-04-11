#ifndef PREDICTION_MODEL_HPP
#define PREDICTION_MODEL_HPP

#include <muse_amcl/data_sources/tf_provider.hpp>
#include <muse_amcl/data_types/data.hpp>
#include <muse_amcl/particle_filter/particle_set.hpp>

#include <muse_amcl/utils/logger.hpp>
#include <memory>

namespace muse_amcl {
class PredictionModel {
public:
    using Ptr = std::shared_ptr<PredictionModel>;

    struct Movement {
        Movement(const double linear_distance,
                 const double angular_distance,
                 const ros::Duration &duration) :
            linear_distance(linear_distance),
            angular_distance(angular_distance),
            prediction_duration(duration)
        {
        }

        Movement() :
            linear_distance(0.0),
            angular_distance(0.0),
            prediction_duration(0.0)

        {
        }
        const double        linear_distance;
        const double        angular_distance;
        const ros::Duration prediction_duration;
    };

    struct Result {
        Result(const double linear_distance_moved,
               const double angular_distance_moved,
               const ros::Duration &prediction_duration,
               const Data::ConstPtr &left_over) :
            movement(linear_distance_moved, angular_distance_moved, prediction_duration),
            left_over(left_over)
        {
        }

        Result(const double linear_distance_moved,
               const double angular_distance_moved,
               const ros::Duration &prediction_duration) :
            movement(linear_distance_moved, angular_distance_moved, prediction_duration)
        {
        }

        Result() = default;

        const Movement       movement;
        const Data::ConstPtr left_over;  /// leftover for prediction, either it could not be
                                         /// predicted until time or there is just a part still
                                         /// open to be used for prediction
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
        tf_provider_ = tf_provider;
        eps_zero_linear_  = nh_private.param(privateParameter("eps_zero_linear"), 1e-4);
        eps_zero_angular_ = nh_private.param(privateParameter("eps_zero_angular"), 1e-4);

        Logger &l = Logger::getLogger();
        l.info("Setup", "PredictionModel:" + name_);
        l.info("eps_zero_linear_='" + std::to_string(eps_zero_linear_) + "'", "PredictionModel:" + name_);
        l.info("eps_zero_angular_='" + std::to_string(eps_zero_angular_) + "'", "PredictionModel:" + name_);

        doSetup(nh_private);
    }


    Result predict(const Data::ConstPtr &data,
                   const ros::Time      &until,
                   ParticleSet::Poses    poses)
    {
        if(until < data->getTimeFrame().end) {
            return Result(0, 0, ros::Duration(0.0), data);
        } else {
            return doPredict(data, until, poses);
        }
    }

protected:
    std::string         name_;
    TFProvider::Ptr     tf_provider_;
    double              eps_zero_linear_;
    double              eps_zero_angular_;

    virtual void doSetup(ros::NodeHandle &nh) = 0;
    virtual Result doPredict(const Data::ConstPtr &data,
                             const ros::Time      &until,
                             ParticleSet::Poses    poses) = 0;

    inline std::string privateParameter(const std::string &name)
    {
        return name_ + "/" + name;
    }


};
}

#endif // PREDICTION_MODEL_HPP
