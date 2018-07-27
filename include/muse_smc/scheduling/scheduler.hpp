#ifndef MUSE_SMC_SCHEDULING_HPP
#define MUSE_SMC_SCHEDULING_HPP

#include <muse_smc/resampling/resampling.hpp>
#include <muse_smc/update/update.hpp>
#include <cslibs_time/rate.hpp>

#include <memory>

namespace muse_smc {
template<typename state_space_description_t, typename data_t>
class Scheduler {
public:
    using Ptr = std::shared_ptr<Scheduler>;

    using id_t         = std::size_t;
    using update_t     = Update<state_space_description_t, data_t>;
    using resampling_t = Resampling<state_space_description_t, data_t>;
    using sample_set_t = SampleSet<state_space_description_t>;

    virtual ~Scheduler() = default;

    virtual bool apply(typename update_t::Ptr     &u,
                       typename sample_set_t::Ptr &s) = 0;

    virtual bool apply(typename resampling_t::Ptr &r,
                       typename sample_set_t::Ptr &s) = 0;
};
}

#endif // MUSE_SMC_SCHEDULING_HPP
