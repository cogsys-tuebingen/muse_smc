#ifndef PREDICTION_HPP
#define PREDICTION_HPP

#include <muse/data/data.hpp>
#include <muse/samples/sample_set.hpp>
#include <muse/prediction/prediction_model.hpp>

namespace muse {
template<typename sample_t>
class Prediction {
public:
    using Ptr = std::shared_ptr<Prediction>;
    using predition_model_t = predition_model_t<sample_t>;
    using sample_set_t = SampleSet<sample_t>;

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

    inline PredictionModel::Result operator ()
        (const Time &until, sample_set_t::state_iterator_t states)
    {
        return model_->predict(data_, until, states);
    }

    inline PredictionModel::Result::Ptr apply(const Time  &until,
                                              sample_set_t::state_iterator_t states)
    {
        return model_->predict(data_, until, states);
    }

    inline const Time & getStamp() const
    {
        return data_->getTimeFrame().start;
    }

    inline const std::string& getModelName() const
    {
        return model_->getName();
    }

    inline PredictionModel::Ptr getModel() const
    {
        return model_;
    }

private:
    Data::ConstPtr          data_;
    PredictionModel::Ptr    model_;
};
}


#endif // PREDICTION_HPP
