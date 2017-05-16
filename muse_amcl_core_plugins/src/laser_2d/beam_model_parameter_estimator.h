#ifndef BEAM_MODEL_PARAMETER_ESTIMATOR_H
#define BEAM_MODEL_PARAMETER_ESTIMATOR_H

#include <atomic>
#include <thread>


class BeamModelParameterEstimator
{
public:
    BeamModelParameterEstimator(const std::size_t max_iterations = 0);

    bool isRunning() const;

    void setInitialParameters(const double z_hit,
                              const double z_max,
                              const double z_short,
                              const double z_rand,
                              const double sigma_hit,
                              const double lambda_short);

    void insert(const double z,
                const double z_bar);

private:
    std::atomic_bool        running_;
    std::atomic_bool        stop_;
    std::thread             worker_thread_;
    std::size_t             max_iterations_;

    double z_hit_;
    double z_short_;
    double z_max_;
    double z_rand_;
    double sigma_hit_;
    double lambda_short_;


};

#endif // BEAM_MODEL_PARAMETER_ESTIMATOR_H
