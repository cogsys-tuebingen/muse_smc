#include <muse_amcl/math/random.hpp>
#include <iostream>
#include <vector>

int main(int argc, char *argv[])
{

    /// test random vecotor generation
    const std::size_t size = 10;
    {
        std::cout << "multimodal: " << std::endl;
        muse_amcl::math::random::Uniform<1> rng(0.0, 1.0);
        std::size_t k = size;
        std::vector<double> u(size, std::pow(rng.get(), 1.0 / k));
        {
            auto u_it = u.rbegin();
            auto u_it_last = u_it;
            auto u_end = u.rend();
            ++u_it;
            while(u_it != u_end) {
                *u_it = *u_it_last * std::pow(rng.get(), 1.0 / k);
                u_it_last = u_it;
                ++u_it;
                --k;
            }
            for(auto u_it : u)
                std::cout << u_it << " ";
            std::cout << std::endl;
        }
    }
    {
        std::cout << "stratified: " << std::endl;
        std::vector<double> u(size);
        std::cout << std::endl;
        muse_amcl::math::random::Uniform<1> rng(0.0, 1.0);
        for(std::size_t i = 0 ; i < size ; ++i) {
            u[i] = (i + rng.get()) / size;
            std::cout << u[i] << " ";
        }
        std::cout << std::endl;
    }
    {
        std::cout << "systematic: " << std::endl;
        std::vector<double> u(size);
        std::cout << std::endl;
        muse_amcl::math::random::Uniform<1> rng(0.0, 1.0);
        double u_static = rng.get();
        for(std::size_t i = 0 ; i < size ; ++i) {
            u[i] = (i + u_static) / size;
            std::cout << u[i] << " ";
        }
        std::cout << std::endl;
    }

    /// test resampling

    return 0;
}


