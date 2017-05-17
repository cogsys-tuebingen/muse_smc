#ifndef BEAM_MODEL_PARAMETER_ESTIMATOR_H
#define BEAM_MODEL_PARAMETER_ESTIMATOR_H

#include <atomic>
#include <thread>
#include <vector>

namespace muse_amcl {
class BeamModelParameterEstimator
{
public:
    struct Parameters {
        double z_hit;
        double z_max;
        double z_short;
        double z_rand;
        double sigma_hit;
        double denominator_hit;
        double lambda_short;
        double range_max;
    };

    BeamModelParameterEstimator(const Parameters &parameters,
                                const std::size_t max_iterations = 0);

    void setMeasurements(const std::vector<double> z,
                         const std::vector<double> z_bar);

    void getParameters(Parameters &parameters) const;

private:
    std::atomic_bool        running_;
    std::atomic_bool        stop_;
    std::thread             worker_thread_;
    std::size_t             max_iterations_;

    std::vector<double>     z_;
    std::vector<double>     z_bar_;


    Parameters              parameters_;
    Parameters              parameters_working_copy_;

    void run();

};
}

#endif // BEAM_MODEL_PARAMETER_ESTIMATOR_H
