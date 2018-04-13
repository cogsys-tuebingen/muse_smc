#ifndef PREDICTION_MODEL_HPP
#define PREDICTION_MODEL_HPP

#include <muse_smc/samples/sample_set.hpp>
#include <muse_smc/state_space/state_space.hpp>

#include <memory>

namespace muse_smc {
template<typename state_space_description_t, typename data_t>
class PredictionModel {
public:
    using Ptr           = std::shared_ptr<PredictionModel>;
    using sample_t      = typename state_space_description_t::state_t;
    using sample_set_t  = SampleSet<state_space_description_t>;
    using state_space_t = StateSpace<state_space_description_t>;

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

    inline static const std::string Type()
    {
        return "muse_smc::PredictionModel";
    }

    inline const std::string& getName() const
    {
        return name_;
    }

    inline void setName(const std::string  &name)
    {
        name_ = name;
    }

    inline std::size_t getId() const
    {
        return id_;
    }

    inline void setId(const std::size_t id)
    {
        id_ = id;
    }

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

protected:
    std::string         name_;
    std::size_t         id_;
};
}

#endif // PREDICTION_MODEL_HPP
