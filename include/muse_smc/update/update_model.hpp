#ifndef UPDATE_MODEL_HPP
#define UPDATE_MODEL_HPP

#include <memory>

#include <muse_smc/state_space/state_space.hpp>
#include <muse_smc/samples/sample_set.hpp>

namespace muse_smc {
template<typename state_space_description_t, typename data_t>
class UpdateModel {
public:
    using Ptr           = std::shared_ptr<UpdateModel>;
    using sample_t      = typename state_space_description_t::sample_t;
    using sample_set_t  = SampleSet<state_space_description_t>;
    using state_space_t = StateSpace<state_space_description_t>;

    UpdateModel()
    {
    }

    virtual ~UpdateModel() = default;

    virtual std::size_t getId() const = 0;
    virtual const std::string getName() const = 0;
    virtual void apply(const typename data_t::ConstPtr          &data,
                       const typename state_space_t::ConstPtr   &state_space,
                       typename sample_set_t::weight_iterator_t  weights) = 0;
};
}

#endif // UPDATE_MODEL_HPP
