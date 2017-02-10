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
            return lhs.getStamp() < rhs.getStamp();
        }
        bool operator()( const Prediction::Ptr& lhs,
                         const Prediction::Ptr& rhs ) const
        {
            return lhs->getStamp() < rhs->getStamp();
        }
    };

    struct Greater {
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


    Prediction(const Data::ConstPtr       &data,
               const PredictionModel::Ptr &model) :
        data_(data),
        model_(model)
    {
    }

    inline PredictionModel::Movement operator ()
        (const ros::Time &until, ParticleSet::Poses poses)
    {
        PredictionModel::Result model_response = model_->predict(data_, until, poses);
        data_ = model_response.left_over;
        return model_response.movement;
    }

    inline PredictionModel::Movement
        apply(const ros::Time &until, ParticleSet::Poses poses)
    {
        PredictionModel::Result model_response = model_->predict(data_, until, poses);
        data_ = model_response.left_over;
        return model_response.movement;
    }

    inline bool isDone() const
    {
        return data_.get() == nullptr;
    }

    inline const ros::Time & getStamp() const
    {
        return data_->getTimeFrame().begin;
    }

private:
    Data::ConstPtr          data_;
    PredictionModel::Ptr    model_;
};
}


#endif // PREDICTION_HPP
