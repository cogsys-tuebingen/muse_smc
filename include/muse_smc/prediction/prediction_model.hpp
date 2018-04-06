#ifndef PREDICTION_MODEL_HPP
#define PREDICTION_MODEL_HPP

#include <cslibs_plugins_data/data.hpp>
#include <muse_smc/samples/sample_set.hpp>

#include <memory>

namespace muse_smc {
template<typename state_space_description_t>
class PredictionModel {
public:
    using Ptr           = std::shared_ptr<PredictionModel>;
    using sample_t      = typename state_space_description_t::state_t;
    using sample_set_t  = SampleSet<state_space_description_t>;

    struct Result {
        using Ptr = std::shared_ptr<Result>;
        using ConstPtr = std::shared_ptr<Result const>;

        Result() = default;
        virtual ~Result() = default;

        Result(const cslibs_plugins_data::Data::ConstPtr &applied) :
            applied(applied)
        {
        }

        Result(const cslibs_plugins_data::Data::ConstPtr &applied,
               const cslibs_plugins_data::Data::ConstPtr &left_to_apply) :
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

        const cslibs_plugins_data::Data::ConstPtr applied;
        const cslibs_plugins_data::Data::ConstPtr left_to_apply;

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

    virtual typename Result::Ptr apply(const cslibs_plugins_data::Data::ConstPtr &data,
                                       const cslibs_time::Time                   &until,
                                       typename sample_set_t::state_iterator_t    states) = 0;

protected:
    std::string         name_;
    std::size_t         id_;

};
}

#endif // PREDICTION_MODEL_HPP
