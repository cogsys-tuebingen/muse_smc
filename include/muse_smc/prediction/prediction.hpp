#ifndef PREDICTION_HPP
#define PREDICTION_HPP

#include <muse_smc/samples/sample_set.hpp>
#include <muse_smc/state_space/state_space.hpp>

#include <cslibs_time/time_frame.hpp>

namespace muse_smc {
template<typename prediction_model_t>
class Prediction {
public:
    using Ptr               = std::shared_ptr<Prediction>;
    using sample_t          = typename prediction_model_t::sample_t;
    using sample_set_t      = typename prediction_model_t::sample_set_t;
    using state_space_t     = typename prediction_model_t::state_space_t;
    using data_t            = typename muse_smc::traits::Data<sample_t>::type;

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
            const auto &lhs_stamp = lhs.getStamp();
            const auto &rhs_stamp = rhs.getStamp();
            return lhs_stamp == rhs_stamp ? lhs.stampReceived() > rhs.stampReceived() :
                                            lhs_stamp > rhs_stamp;
        }
        bool operator()( const Prediction::Ptr& lhs,
                         const Prediction::Ptr& rhs ) const
        {
            const auto &lhs_stamp = lhs->getStamp();
            const auto &rhs_stamp = rhs->getStamp();
            return lhs_stamp == rhs_stamp ? lhs->stampReceived() > rhs->stampReceived() :
                                            lhs_stamp > rhs_stamp;
        }
    };

    Prediction(const typename data_t::ConstPtr       &data,
               const typename prediction_model_t::Ptr &model) :
        data_(data),
        model_(model)
    {
    }

    Prediction(const typename data_t::ConstPtr        &data,
               const typename state_space_t::ConstPtr &state_space,
               const typename prediction_model_t::Ptr  &model) :
        data_(data),
        state_space_(state_space),
        model_(model)
    {
    }

    virtual ~Prediction() = default;

    inline typename prediction_model_t::Result operator ()
        (const cslibs_time::Time &until, typename sample_set_t::state_iterator_t states)
    {
        return state_space_ ?
                    model_->apply(data_, state_space_, until, states) :
                    model_->apply(data_, until, states);
    }

    inline typename prediction_model_t::Result::Ptr apply(const cslibs_time::Time  &until,
                                                         typename sample_set_t::state_iterator_t states)
    {
        return state_space_ ?
                    model_->apply(data_, state_space_, until, states) :
                    model_->apply(data_, until, states);
    }

    inline const cslibs_time::Time& getStamp() const
    {
        return data_->timeFrame().end;
    }

    inline const cslibs_time::TimeFrame &timeFrame() const
    {
        return data_->timeFrame();
    }

    inline cslibs_time::Time const & stampReceived() const
    {
        return data_->stampReceived();
    }

    inline const std::string& getModelName() const
    {
        return model_->getName();
    }

    inline  typename prediction_model_t::Ptr getModel() const
    {
        return model_;
    }

private:
    typename data_t::ConstPtr        data_;
    typename state_space_t::ConstPtr state_space_;
    typename prediction_model_t::Ptr  model_;
};
}

#endif // PREDICTION_HPP
