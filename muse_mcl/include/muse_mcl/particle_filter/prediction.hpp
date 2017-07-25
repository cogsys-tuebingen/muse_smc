#ifndef PREDICTION_HPP
#define PREDICTION_HPP

#include <muse_mcl/data_types/data.hpp>
#include <muse_mcl/plugins/types/provider_data.hpp>

#include <muse_mcl/particle_filter/particle_set.hpp>

#include <muse_mcl/plugins/types/model_prediction.hpp>

namespace muse_mcl {
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
               const ModelPrediction::Ptr &model) :
        data_(data),
        model_(model)
    {
    }

    inline ModelPrediction::Result operator ()
        (const ros::Time &until, ParticleSet::Poses poses)
    {
        return model_->predict(data_, until, poses);
    }

    inline ModelPrediction::Result apply(const ros::Time &until,
                                         ParticleSet::Poses poses)
    {
        return model_->predict(data_, until, poses);
    }

    inline const ros::Time & getStamp() const
    {
        return data_->getTimeFrame().start;
    }

    inline const std::string& getModelName() const
    {
        return model_->getName();
    }

    inline ModelPrediction::Ptr getModel() const
    {
        return model_;
    }

private:
    Data::ConstPtr          data_;
    ModelPrediction::Ptr    model_;
};
}


#endif // PREDICTION_HPP
