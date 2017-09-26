#include <muse_smc/time/time.hpp>
#include <muse_smc/math/random.hpp>

static const int ITERATIONS = 1000000;

void std_exp()
{
    muse_smc::math::random::Uniform<1> rng(-100.0, +100.0);

    double ms = 0;
    double y = 0.0;
    for(int i = 0 ; i < ITERATIONS ; ++i) {
        double x = rng.get();

        muse_smc::Time start = muse_smc::Time::now();
        y = std::exp(x);
        ms += (muse_smc::Time::now() - start).milliseconds();
    }
    std::cout << "std exp took: " << ms / ITERATIONS << "ms" << std::endl;
}

void c_exp()
{
    muse_smc::math::random::Uniform<1> rng(-100.0, +100.0);

    double ms = 0;
    double y = 0.0;
    for(int i = 0 ; i < ITERATIONS ; ++i) {
        double x = rng.get();

        muse_smc::Time start = muse_smc::Time::now();
        y = exp(x);
        ms += (muse_smc::Time::now() - start).milliseconds();
    }
    std::cout << "c   exp took: " << ms / ITERATIONS << "ms" << std::endl;
}

void std_log()
{
    muse_smc::math::random::Uniform<1> rng(-100.0, +100.0);

    double ms = 0;
    double y = 0.0;
    for(int i = 0 ; i < ITERATIONS ; ++i) {
        double x = rng.get();

        muse_smc::Time start = muse_smc::Time::now();
        y = std::log(x);
        ms += (muse_smc::Time::now() - start).milliseconds();
    }
    std::cout << "std  log took: " << ms / ITERATIONS << "ms" << std::endl;
}

void c_log()
{
    muse_smc::math::random::Uniform<1> rng(-100.0, +100.0);

    double ms = 0;
    double y = 0.0;
    for(int i = 0 ; i < ITERATIONS ; ++i) {
        double x = rng.get();

        muse_smc::Time start = muse_smc::Time::now();
        y = log(x);
        ms += (muse_smc::Time::now() - start).milliseconds();
    }
    std::cout << "c   log took: " << ms / ITERATIONS << "ms" << std::endl;
}


int main(int argc, char *argv[])
{
    for(std::size_t i = 0 ; i < 10 ; ++i) {
        std_exp();
        c_exp();
        std_log();
        c_log();
        std::cout << "--------------------------------" << std::endl;
    }
    return 0;
}
