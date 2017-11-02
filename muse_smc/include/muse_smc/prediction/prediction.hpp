#ifndef PREDICTION_HPP
#define PREDICTION_HPP

#include <muse_smc/data/data.hpp>
#include <muse_smc/samples/sample_set.hpp>
#include <muse_smc/prediction/prediction_model.hpp>

namespace muse_smc {
template<typename state_space_description_t>
class Prediction {
public:
    using Ptr = std::shared_ptr<Prediction>;
    using predition_model_t = PredictionModel<state_space_description_t>;
    using sample_set_t = SampleSet<state_space_description_t>;

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


    Prediction(const Data::ConstPtr                  &data,
               const typename predition_model_t::Ptr &model) :
        data_(data),
        model_(model)
    {
    }

    inline typename predition_model_t::Result operator ()
        (const cslibs_time::Time &until, typename sample_set_t::state_iterator_t states)
    {
        return model_->apply(data_, until, states);
    }

    inline typename predition_model_t::Result::Ptr apply(const cslibs_time::Time  &until,
                                                         typename sample_set_t::state_iterator_t states)
    {
        return model_->apply(data_, until, states);
    }

    inline const cslibs_time::Time& getStamp() const
    {
        return data_->getTimeFrame().start;
    }

    inline const std::string& getModelName() const
    {
        return model_->getName();
    }

    inline  typename predition_model_t::Ptr getModel() const
    {
        return model_;
    }

private:
    Data::ConstPtr                     data_;
    typename predition_model_t::Ptr    model_;
};
}


#endif // PREDICTION_HPP
