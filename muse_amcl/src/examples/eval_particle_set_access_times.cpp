#include <muse_amcl/particle_filter/particle_set.hpp>
#include <chrono>
#include <iostream>

double sum_x = 0.0;

void states(muse_amcl::ParticleSet& set)
{
    for(auto &state : set.getPoses()) {
        state.x() = 1.f;
    }

}

void weights(muse_amcl::ParticleSet &set)
{
    std::size_t iteration = 0;
    for(auto &state : set.getWeights()) {
       state = 1.0;
    }
}

void particle(muse_amcl::ParticleSet &set)
{
    std::size_t pos = 0;
    for(auto &p : set.getSamples()) {
        const double w = p.weight_;
        const auto pose = p.pose_;
            if(pose.x() == 0)
                std::cout << "was zero " << pos << std::endl;
            sum_x += pose.x();
            ++pos;
    }
}

int main(int argc, char *argv[])
{

    const std::size_t iterations = 1;

    muse_amcl::Indexation indexation ({0.1, 0.1, 1./18. * M_PI});

    muse_amcl::ParticleSet particles("frame", 500000, indexation);
    auto in = particles.getInsertion();
    while(in.canInsert()) {
        in.insert(muse_amcl::Particle());
    }
    in.close();

    {
        auto start = std::chrono::high_resolution_clock::now();
        for(std::size_t iteration = 0; iteration < iterations; ++iteration) {
            states(particles);
        }
        auto end = std::chrono::high_resolution_clock::now();

        long length_micro_seconds = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count();
        std::cerr << "states foreach: " << (length_micro_seconds / double(iterations)) * 1e-6 << "ms" << std::endl;
    }
    {
        auto start = std::chrono::high_resolution_clock::now();
        for(std::size_t iteration = 0; iteration < iterations; ++iteration) {
            weights(particles);
        }
        auto end = std::chrono::high_resolution_clock::now();

        long length_micro_seconds = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count();
        std::cerr << "weights foreach: " << (length_micro_seconds / double(iterations)) * 1e-6 << "ms" << std::endl;
    }

    {
        auto start = std::chrono::high_resolution_clock::now();
        for(std::size_t iteration = 0; iteration < iterations; ++iteration) {
            particle(particles);
        }
        auto end = std::chrono::high_resolution_clock::now();

        long length_micro_seconds = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count();
        std::cerr << "particle foreach: " << (length_micro_seconds / double(iterations)) * 1e-6 << "ms" << std::endl;
    }

    std::cout << "a happy set of " << sum_x << " particles" << std::endl;

    return 0;
}
