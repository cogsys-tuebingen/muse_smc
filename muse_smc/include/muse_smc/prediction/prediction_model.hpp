#ifndef PREDICTION_MODEL_HPP
#define PREDICTION_MODEL_HPP

#include <muse_smc/data/data.hpp>
#include <muse_smc/samples/sample_set.hpp>


#include <memory>

namespace muse_smc {
template<typename sample_t>
class PredictionModel {
public:
    using sample_set_t = SampleSet<sample_t>;
    using Ptr = std::shared_ptr<PredictionModel<sample_t>>;

    struct Result {
        using Ptr = std::shared_ptr<PredictionModel<sample_t>::Result>;

        Result() = default;

        Result(const Data::ConstPtr &applied) :
            applied(applied)
        {
        }

        Result(const Data::ConstPtr &applied,
               const Data::ConstPtr &left_to_apply) :
            applied(applied),
            left_to_apply(left_to_apply)
        {
        }

        Result(const Result &other) :
            applied(other.applied),
            left_to_apply(other.left_to_apply)
        {
        }

        inline bool success() const
        {
            return static_cast<bool>(applied);
        }

        const Data::ConstPtr applied;
        const Data::ConstPtr left_to_apply;

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

    virtual typename Result::Ptr predict(const Data::ConstPtr                    &data,
                                         const Time                              &until,
                                         typename sample_set_t::state_iterator_t  states) = 0;

protected:
    std::string         name_;
    std::size_t         id_;

};
}

#endif // PREDICTION_MODEL_HPP
