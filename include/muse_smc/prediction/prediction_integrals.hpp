#ifndef PREDICTION_INTEGRALS_HPP
#define PREDICTION_INTEGRALS_HPP

#include <muse_smc/prediction/prediction_integral.hpp>
#include <unordered_map>

namespace muse_smc {
template<typename sample_t>
class PredictionIntegrals
{
public:
    using Ptr                   = std::shared_ptr<PredictionIntegrals>;
    using prediction_model_t    = PredictionModel<sample_t>;
    using prediction_integral_t = PredictionIntegral<sample_t>;
    using id_t                  = std::size_t;
    using map_t = std::unordered_map<id_t, typename prediction_integral_t::Ptr>;


    PredictionIntegrals(const typename prediction_integral_t::Ptr &global_integral) :
        global_accumulator_(global_integral)
    {
    }

    virtual ~PredictionIntegrals() = default;

    inline void set(const typename prediction_integral_t::Ptr &accumulator,
                    const id_t id)
    {
        accumulators_[id] = accumulator;
    }

    inline typename prediction_integral_t::ConstPtr get(const id_t id) const
    {
        return accumulators_[id];
    }

    inline typename prediction_integral_t::ConstPtr get() const
    {
        return global_accumulator_;
    }

    inline void info() const
    {
        global_accumulator_->info();
    }

    inline bool thresholdExceeded(const id_t id) const
    {
        return accumulators_.at(id)->thresholdExceeded();
    }

    inline bool isZero(const id_t id) const
    {
        return accumulators_.at(id)->isZero();
    }

    inline bool thresholdExceeded() const
    {
        return global_accumulator_->thresholdExceeded();
    }

    inline bool isZero() const
    {
        return global_accumulator_->isZero();
    }

    inline void reset(const id_t id)
    {
        accumulators_[id]->reset();
    }

    inline void reset()
    {
        global_accumulator_->reset();
    }

    inline void resetAll()
    {
        for(auto &a : accumulators_) {
            a.second->reset();
        }
        global_accumulator_->reset();
    }

    inline void add(const typename prediction_model_t::Result::ConstPtr &step)
    {
        for(auto &a : accumulators_) {
            a.second->add(step);
        }
        global_accumulator_->add(step);
    }

protected:
    typename prediction_integral_t::Ptr global_accumulator_;
    map_t                               accumulators_;
};
}


#endif // PREDICTION_INTEGRALS_HPP
