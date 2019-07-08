#ifndef PREDICTION_MODEL_HPP
#define PREDICTION_MODEL_HPP

#include <muse_smc/samples/sample_set.hpp>
#include <muse_smc/state_space/state_space.hpp>

#include <memory>

namespace muse_smc {
template<typename sample_t>
class PredictionModel {
public:
    using Ptr           = std::shared_ptr<PredictionModel>;
    using data_t        = typename traits::Data<sample_t>::type;
    using sample_set_t  = SampleSet<sample_t>;
    using state_space_t = StateSpace<sample_t>;

    struct Result {
        using Ptr      = std::shared_ptr<Result>;
        using ConstPtr = std::shared_ptr<Result const>;

        Result() = default;
        virtual ~Result() = default;

        Result(const typename data_t::ConstPtr &applied) :
            applied(applied)
        {
        }

        Result(const typename data_t::ConstPtr &applied,
               const typename data_t::ConstPtr &left_to_apply) :
            applied(applied),
            left_to_apply(left_to_apply)
        {
        }

        inline bool success() const
        {
            return static_cast<bool>(applied);
        }

        template<typename T>
        bool isType() const
        {
            const T *t = dynamic_cast<const T*>(this);
            return t != nullptr;
        }

        template<typename T>
        T const & as() const
        {
            return dynamic_cast<const T&>(*this);
        }

        const typename data_t::ConstPtr applied;
        const typename data_t::ConstPtr left_to_apply;
    };

    PredictionModel() = default;
    virtual ~PredictionModel() = default;

    virtual typename Result::Ptr apply(const typename data_t::ConstPtr          &data,
                                       const cslibs_time::Time                  &until,
                                       typename sample_set_t::state_iterator_t   states) = 0;

    virtual typename Result::Ptr apply(const typename data_t::ConstPtr          &data,
                                       const typename state_space_t::ConstPtr   &state_space,
                                       const cslibs_time::Time                  &until,
                                       typename sample_set_t::state_iterator_t   states)
    {
        return apply(data, until, states);
    }
};
}

#endif // PREDICTION_MODEL_HPP
