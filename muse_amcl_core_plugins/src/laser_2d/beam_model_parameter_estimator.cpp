#include "beam_model_parameter_estimator.h"

using namespace muse_amcl;

#include <cmath>

BeamModelParameterEstimator::BeamModelParameterEstimator(const Parameters &parameters,
                                                         const std::size_t max_iterations) :
    running_(false),
    stop_(false),
    max_iterations_(max_iterations),
    parameters_(parameters),
    parameters_working_copy_(parameters)
{
}

void BeamModelParameterEstimator::setMeasurements(const std::vector<double> z,
                                                  const std::vector<double> z_bar)
{
    if(running_)
        return;

    z_ = z;
    z_bar_ = z_bar;

    /// kick off the thread here

}

void BeamModelParameterEstimator::getParameters(Parameters &parameters) const
{
    parameters = parameters_;
}


void BeamModelParameterEstimator::run()
{
    /// probabilities
    auto p_hit = [this](const double ray_range, const double map_range) {
        const double dz = ray_range - map_range;
        return parameters_working_copy_.z_hit * std::exp(-dz * dz * parameters_working_copy_.denominator_hit);
    };
    auto p_short = [this](const double ray_range, const double map_range) {
        if(ray_range < map_range) {
            return parameters_working_copy_.z_short *
                   (1.0 / (1.0 - std::exp(-parameters_working_copy_.lambda_short  * map_range))) *
                    parameters_working_copy_.lambda_short * exp(-parameters_working_copy_.lambda_short * ray_range);
        }
        return 0.0;
    };
    auto p_max = [this](const double ray_range)
    {
        if(ray_range >= parameters_working_copy_.range_max)
            return parameters_working_copy_.z_max * 1.0;
        return 0.0;
    };
    auto p_random = [this](const double ray_range)
    {
        const double p_rand = parameters_working_copy_.z_rand * 1.0 / parameters_working_copy_.range_max;
        if(ray_range < parameters_working_copy_.range_max)
            return p_rand;
        return 0.0;
    };
    auto probability = [p_hit, p_short, p_max, p_random] (const double ray_range, const double map_range)
    {
        return p_hit(ray_range, map_range) + p_short(ray_range, map_range) + p_max(ray_range) + p_random(ray_range);
    };


//    running_ = true;
//    auto terminate = [&stop_, &max_iterations_] (std::size_t iteration) {
//        return stop_ || (max_iterations_ > 0 && iteration > max_iterations_);
//    };

//    /// beam model functions

//    /// here the estimation goes
//    while(!terminate()) {




//    }

    parameters_ = parameters_working_copy_;
    running_ = false;
}
