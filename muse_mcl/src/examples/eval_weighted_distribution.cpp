#include "../../include/muse_mcl/math/random.hpp"
#include "../../include/muse_mcl/math/weighted_distribution.hpp"
#include <vector>
#include <iostream>

int main(int argc, char *argv[])
{

    const std::size_t size = 100;
    muse_mcl::math::random::Uniform<1> rng(0.0, 1.0);
    std::vector<double> u;
    std::vector<Eigen::Vector2d> samples;
    double sum = 0.0;
    for(std::size_t i = 0 ; i < size ; ++i) {
        double r = rng.get();
        u.emplace_back(r);
        sum += r;
        samples.emplace_back(Eigen::Vector2d(static_cast<double>(i), static_cast<double>(i)));

    }
    for(double &w : u) {
        w /= sum;
    }



    muse_mcl::math::statistic::WeightedDistribution<2> w;
    for(std::size_t i = 0 ; i < size ; ++i) {
        w.add(samples[i], u[i]);
        std::cout << u[i] << std::endl;
    }

    std::cout << w.getSampleCount() << std::endl;
    std::cout << w.getWeight() << std::endl;


    return 0;
}
