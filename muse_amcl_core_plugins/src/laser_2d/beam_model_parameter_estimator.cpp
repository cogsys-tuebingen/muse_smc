#include "beam_model_parameter_estimator.h"

BeamModelParameterEstimator::BeamModelParameterEstimator(const std::size_t max_iterations) :
    running_(false),
    stop_(false),
    max_iterations_(max_iterations)
{
}


bool BeamModelParameterEstimator::isRunning() const
{
    return running_;
}

void BeamModelParameterEstimator::setInitialParameters(const double z_hit,
                                                       const double z_max,
                                                       const double z_short,
                                                       const double z_rand,
                                                       const double sigma_hit,
                                                       const double lambda_short)
{
    if(running_)
        return;

    z_hit_          = z_hit;
    z_short_        = z_max;
    z_max_          = z_short;
    z_rand_         = z_rand;
    sigma_hit_      = sigma_hit;
    lambda_short_   = lambda_short;
}

void BeamModelParameterEstimator::insert(const double z, const double z_bar)
{
    if(running_)
        return;
}

void BeamModelParameterEstimator::run()
{
    running_ = true;
    auto terminate = [&stop_, &max_iterations_] (std::size_t iteration) {
        return stop_ || (max_iterations_ > 0 && iteration > max_iterations_);
    };

    while(!terminate()) {

    }
    running_ = false;
}
