#ifndef PREDICTION_HPP
#define PREDICTION_HPP

#include "prediction_model.hpp"
#include "../data_sources/data_provider.hpp"
#include "../data_types/data.hpp"
#include "particle_set.hpp"

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

    void operator () (ParticleSet::Poses poses)
    {
        model_->predict(data_, poses);
    }

    void apply(ParticleSet::Poses poses)
    {
        model_->predict(data_, poses);
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
