#ifndef UPDATE_MODEL_HPP
#define UPDATE_MODEL_HPP

#include <memory>

#include <muse_smc/update/update.hpp>

namespace muse_smc {
template<typename smc_t>
class UpdateModel {
public:
    using Ptr           = std::shared_ptr<UpdateModel>;
    using sample_t      = typename smc_t::sample_t;
    using sample_set_t  = typename smc_t::sample_set_t;
    using state_space_t = typename smc_t::state_space_t;
    using update_t      = Update<UpdateModel>;

    using data_t        = typename traits::Data<sample_t>::type;
    using covariance_t  = typename traits::Covariance<sample_t>::type;

    virtual std::size_t getId() const = 0;
    virtual const std::string getName() const = 0;
    virtual void apply(const typename data_t::ConstPtr          &data,
                       const typename state_space_t::ConstPtr   &state_space,
                       typename sample_set_t::weight_iterator_t  weights) = 0;
};
}

#endif // UPDATE_MODEL_HPP
