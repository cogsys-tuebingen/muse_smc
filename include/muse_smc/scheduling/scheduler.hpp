#ifndef MUSE_SMC_SCHEDULING_HPP
#define MUSE_SMC_SCHEDULING_HPP

#include <muse_smc/resampling/resampling.hpp>
#include <muse_smc/update/update.hpp>
#include <cslibs_time/rate.hpp>

#include <memory>

namespace muse_smc {
template<typename smc_t>
class Scheduler {
public:
    using Ptr = std::shared_ptr<Scheduler>;

    using id_t         = std::size_t;
    using sample_t     = typename smc_t::sample_t;
    using update_t     = typename smc_t::update_t;
    using resampling_t = typename smc_t::resampling_t;
    using sample_set_t = typename smc_t::sample_set_t;
    using data_t       = typename traits::Data<sample_t>::type;

    virtual bool apply(typename update_t::Ptr     &u,
                       typename sample_set_t::Ptr &s) = 0;

    virtual bool apply(typename resampling_t::Ptr &r,
                       typename sample_set_t::Ptr &s) = 0;

protected:
    Scheduler() = default;
    virtual ~Scheduler() = default;
};
}

#endif // MUSE_SMC_SCHEDULING_HPP
