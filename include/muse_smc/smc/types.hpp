#ifndef MUSE_SMC_TYPES_HPP
#define MUSE_SMC_TYPES_HPP

/// PROJECT
#include <muse_smc/prediction/prediction_integrals.hpp>
#include <muse_smc/prediction/prediction.hpp>
#include <muse_smc/update/update_model.hpp>
#include <muse_smc/sampling/normal.hpp>
#include <muse_smc/sampling/uniform.hpp>
#include <muse_smc/samples/sample_set.hpp>
#include <muse_smc/resampling/resampling.hpp>
#include <muse_smc/scheduling/scheduler.hpp>
#include <muse_smc/smc/state_publisher.hpp>
#include <muse_smc/smc/traits/sample.hpp>
#include <muse_smc/smc/request_state_initilization.hpp>
#include <muse_smc/smc/request_uniform_initilization.hpp>

#include <muse_smc/state_space/state_space_provider.hpp>

namespace muse_smc {
template<typename Sample_T>
struct Types {
    /// filter specific type defs
    using sample_t              = Sample_T;
    using type_t                = Types<sample_t>;
    using sample_set_t          = SampleSet<sample_t>;

    using state_space_provider_t = StateSpaceProvider<sample_t>;
    using state_space_t          = StateSpace<sample_t>;

    using update_model_t        = UpdateModel<type_t>;
    using update_t              = typename update_model_t::update_t;

    using prediction_model_t    = PredictionModel<sample_set_t, typename traits::Data<Sample_T>::type, state_space_t>;
    using prediction_result_t   = typename prediction_model_t::Result;
    using prediction_t          = Prediction<prediction_model_t>;
    using prediction_integral_t = PredictionIntegral<prediction_result_t>;
    using prediction_integrals_t= PredictionIntegrals<prediction_result_t>;

    using scheduler_t           = Scheduler<type_t>;
    using resampling_t          = Resampling<type_t>;

    using time_t                = cslibs_time::Time;
    using duration_t            = cslibs_time::Duration;
    using state_t               = typename traits::State<sample_t>::type;
    using covariance_t          = typename traits::Covariance<sample_t>::type;

    using request_state_initialization_t = RequestStateInitialization<time_t, state_t,covariance_t>;
    using request_uniform_initialization_t = RequestUniformInitialization<time_t>;

    using normal_sampling_t     = NormalSampling<sample_t>;
    using uniform_sampling_t    = UniformSampling<sample_t>;

    using filter_state_t        = StatePublisher<sample_t>;

};
}

#endif // MUSE_SMC_TYPES_HPP