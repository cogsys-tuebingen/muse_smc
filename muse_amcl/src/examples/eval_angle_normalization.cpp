#include <chrono>
#include <random>
#include <iostream>

namespace f {
const double _2_M_PI = 2 * M_PI;
const double _1_2_M_PI = 1 / _2_M_PI;
double normalize(const double _angle)
{
    return _angle - _2_M_PI * floor( _angle * _1_2_M_PI );
}
}

namespace l {
double normalize(double z)
{
    double r = z;
    while(r < -M_PI)
        r += 2 * M_PI;
    while(r >= M_PI)
        r -= 2 * M_PI;
    return r;
}
}

namespace a {
double normalize(double z)
{
    return atan2(sin(z),cos(z));
}
}


int main(int argc, char *argv[])
{
    std::size_t iterations = (std::size_t) 1e6;

    {
        std::default_random_engine e(0);
        std::uniform_real_distribution<double> uniform_dist(-100.0 * M_PI, +100.0 * M_PI);

        auto start = std::chrono::high_resolution_clock::now();
        for(std::size_t i = 0 ; i < iterations ; ++i) {
            f::normalize(uniform_dist(e));
        }
        auto end = std::chrono::high_resolution_clock::now();

        long length_micro_seconds = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count();
        std::cerr << "       floor: " << (length_micro_seconds / double(iterations)) * 1e-6 << "ms" << std::endl;
    }
    {
        std::default_random_engine e(0);
        std::uniform_real_distribution<double> uniform_dist(-100.0 * M_PI, +100.0 * M_PI);

        auto start = std::chrono::high_resolution_clock::now();
        for(std::size_t i = 0 ; i < iterations ; ++i) {
            l::normalize(uniform_dist(e));
        }
        auto end = std::chrono::high_resolution_clock::now();

        long length_micro_seconds = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count();
        std::cerr << "       loop: " << (length_micro_seconds / double(iterations)) * 1e-6 << "ms" << std::endl;

    }
    {
        std::default_random_engine e(0);
        std::uniform_real_distribution<double> uniform_dist(-100.0 * M_PI, +100.0 * M_PI);

        auto start = std::chrono::high_resolution_clock::now();
        for(std::size_t i = 0 ; i < iterations ; ++i) {
            a::normalize(uniform_dist(e));
        }
        auto end = std::chrono::high_resolution_clock::now();

        long length_micro_seconds = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count();
        std::cerr << "       atan: " << (length_micro_seconds / double(iterations)) * 1e-6 << "ms" << std::endl;

    }

    return 0;
}
