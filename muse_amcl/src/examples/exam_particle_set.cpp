#include <muse_amcl/pf/particle_set.hpp>

#include <chrono>
#include <iostream>

void states(muse::ParticleSet& set)
{
    for(auto &state : set.getPoses()) {
        state.getOrigin().m_floats[0] = 1.f;
    }
}

void weights(muse::ParticleSet &set)
{
    for(auto &state : set.getWeigts()) {
       state = 1.0;
    }
}

void particle(muse::ParticleSet &set)
{
    for(auto &p : set.getParticles()) {
        p.weight = 0.0;
    }
}

int main(int argc, char *argv[])
{

    const std::size_t iterations = 50;

    muse::ParticleSet particles(500000);

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
