#include <muse_mcl/math/angular_distribution.hpp>
#include <muse_mcl/math/random.hpp>

using namespace muse_mcl;

int main(int argc, char *argv[])
{
    using sample = math::statistic::AngularDistribution<2>::VectorType;

    math::statistic::AngularDistribution<2> angles;

    math::random::Uniform<1> u(-0.5, 0.5);
    for(std::size_t i = 0 ; i < 10000 ; ++i) {
        angles.add(sample(M_PI - u.get(), M_PI + u.get()));
    }

    std::cout << angles.getMean() << std::endl;
    std::cout << angles.getCovariance() << std::endl;

    return 0;
}
