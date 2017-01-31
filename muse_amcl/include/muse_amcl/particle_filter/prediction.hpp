#ifndef PREDICTION_HPP
#define PREDICTION_HPP

#include <muse_amcl/data_types/data.hpp>
#include <muse_amcl/data_sources/data_provider.hpp>
#include <muse_amcl/particle_filter/particle_set.hpp>
#include <muse_amcl/particle_filter/prediction_model.hpp>

namespace muse_amcl {
class Prediction {
public:
    using Ptr = std::shared_ptr<Prediction>;

    struct Less {
        bool operator()( const Prediction& lhs,
                         const Prediction& rhs ) const
        {
            return lhs.getStamp() > rhs.getStamp();
        }
        bool operator()( const Prediction::Ptr& lhs,
                         const Prediction::Ptr& rhs ) const
        {
            return lhs->getStamp() > rhs->getStamp();
        }
    };



    Prediction(const ros::Time            &stamp,
               const Data::ConstPtr       &data,
               const PredictionModel::Ptr &model) :
        stamp_(stamp),
        data_(data),
        model_(model)
    {
    }

    inline PredictionModel::Result operator ()
        (const ros::Time &until, ParticleSet::Poses poses)
    {
        return model_->predict(data_, until, poses);
    }

    inline PredictionModel::Result
        apply(const ros::Time &until, ParticleSet::Poses poses)
    {
        return model_->predict(data_, until, poses);
    }

    ros::Time getStamp() const
    {
        return stamp_;
    }

private:
    const ros::Time      stamp_;
    const Data::ConstPtr data_;
    PredictionModel::Ptr model_;

};
}


#endif // PREDICTION_HPP
