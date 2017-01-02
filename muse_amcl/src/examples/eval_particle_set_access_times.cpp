#include <muse_amcl/particle_filter/particle_set.hpp>
#include <chrono>
#include <iostream>

void states(muse_amcl::ParticleSet& set)
{
    for(auto &state : set.getPoses()) {
        state.origin().m_floats[0] = 1.f;
    }
}

void weights(muse_amcl::ParticleSet &set)
{
    for(auto &state : set.getWeights()) {
       state = 1.0;
    }
}

void particle(muse_amcl::ParticleSet &set)
{
    for(auto &p : set.getParticles()) {
        p.weight_ = 0.0;
    }
}

int main(int argc, char *argv[])
{

    const std::size_t iterations = 50;

    muse_amcl::ParticleSet particles(500000);

    {
        auto start = std::chrono::high_resolution_clock::now();
        for(std::size_t iteration = 0; iteration < iterations; ++iteration) {
            states(particles);
        }
        auto end = std::chrono::high_resolution_clock::now();

        long length_micro_seconds = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count();
        std::cerr << "       foreach: " << (length_micro_seconds / double(iterations)) * 1e-6 << "ms" << std::endl;
    }
    {
        auto start = std::chrono::high_resolution_clock::now();
        for(std::size_t iteration = 0; iteration < iterations; ++iteration) {
            weights(particles);
        }
        auto end = std::chrono::high_resolution_clock::now();

        long length_micro_seconds = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count();
        std::cerr << "       foreach: " << (length_micro_seconds / double(iterations)) * 1e-6 << "ms" << std::endl;
    }

    {
        auto start = std::chrono::high_resolution_clock::now();
        for(std::size_t iteration = 0; iteration < iterations; ++iteration) {
            particle(particles);
        }
        auto end = std::chrono::high_resolution_clock::now();

        long length_micro_seconds = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count();
        std::cerr << "       foreach: " << (length_micro_seconds / double(iterations)) * 1e-6 << "ms" << std::endl;
    }

    return 0;
}
