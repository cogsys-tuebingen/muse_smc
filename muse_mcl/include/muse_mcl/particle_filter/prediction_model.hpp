#ifndef PREDICTION_MODEL_HPP
#define PREDICTION_MODEL_HPP

#include <muse_mcl/data_sources/tf_provider.hpp>
#include <muse_mcl/data_types/data.hpp>
#include <muse_mcl/particle_filter/particle_set.hpp>

#include <muse_mcl/utils/logger.hpp>
#include <memory>

namespace muse_mcl {
class PredictionModel {
public:
    using Ptr = std::shared_ptr<PredictionModel>;

    struct Result {
        Result() :
            linear_distance_abs(0.0),
            angular_distance_abs(0.0)
        {
        }

        Result(const double linear_distance_abs,
               const double angular_distance_abs,
               const Data::ConstPtr &applied) :
            linear_distance_abs(linear_distance_abs),
            angular_distance_abs(angular_distance_abs),
            applied(applied)
        {
        }

        Result(const double linear_distance_abs,
                const double angular_distance_abs,
                const Data::ConstPtr &applied,
                const Data::ConstPtr &left_to_apply) :
            linear_distance_abs(linear_distance_abs),
            angular_distance_abs(angular_distance_abs),
            applied(applied),
            left_to_apply(left_to_apply)
        {
        }

        inline bool success() const
        {
            return static_cast<bool>(applied);
        }

        const double linear_distance_abs;
        const double angular_distance_abs;

        const Data::ConstPtr applied;
        const Data::ConstPtr left_to_apply;

    };

    PredictionModel()
    {
    }

    virtual ~PredictionModel()
    {
    }

    inline static const std::string Type()
    {
        return "muse_mcl::PredictionModel";
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

        //// drop if too old
        if(until < data->getTimeFrame().end) {
            return Result();
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
