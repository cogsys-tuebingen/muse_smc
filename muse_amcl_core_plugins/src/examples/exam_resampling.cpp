#include <muse_amcl/math/random.hpp>
#include <muse_amcl/particle_filter/particle_set.hpp>
#include <iostream>
#include <vector>
#include "../resampling/multinomial.h"
#include "../resampling/stratified.h"
#include "../resampling/systematic.h"
#include "../resampling/wheel_of_fortune.h"
#include "../resampling/residual.h"

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
    muse_amcl::ParticleSet ps("frame", 10);
    auto &particles = ps.getParticles();
    {
        /// random weights for the particles
        muse_amcl::math::random::Uniform<1> rng(0.0, 1.0);
        double nw = 0.0;
        std::size_t i = 0;
        for(auto &p : particles) {
            p.weight_ = rng.get();
            nw += p.weight_;
            p.pose_.origin().setX(i);
            ++i;
        }
        for(auto &p : particles) {
            p.weight_ /= nw;
        }
    }
    {
        std::cout << "original particle set" << std::endl;
        for(auto &p : particles) {
            std::cout << p.pose_.origin().x() << " " << p.weight_ << std::endl;
        }
    }
    {
        /// multinomial
        auto ps_copy = ps;
        muse_amcl::Multinomial m;
        m.apply(ps_copy);
        std::cout << "multinomial particle set" << std::endl;
        for(auto &p : ps_copy.getParticles()) {
            std::cout << p.pose_.origin().x() << " " << p.weight_ << std::endl;
        }
        std::cout << "----------------" << std::endl;
    }
    {
        /// stratified
        auto ps_copy = ps;
        muse_amcl::Stratified m;
        m.apply(ps_copy);
        std::cout << "stratified particle set" << std::endl;
        for(auto &p : ps_copy.getParticles()) {
            std::cout << p.pose_.origin().x() << " " << p.weight_ << std::endl;
        }
        std::cout << "----------------" << std::endl;
    }
    {
        /// systematic
        auto ps_copy = ps;
        muse_amcl::Systematic m;
        m.apply(ps_copy);
        std::cout << "systematic particle set" << std::endl;
        for(auto &p : ps_copy.getParticles()) {
            std::cout << p.pose_.origin().x() << " " << p.weight_ << std::endl;
        }
        std::cout << "----------------" << std::endl;
    }
    {
        /// residual
        auto ps_copy = ps;
        muse_amcl::Residual m;
        m.apply(ps_copy);
        std::cout << "residual particle set" << std::endl;
        for(auto &p : ps_copy.getParticles()) {
            std::cout << p.pose_.origin().x() << " " << p.weight_ << std::endl;
        }
        std::cout << "----------------" << std::endl;
    }
    {
        /// kld 2D
    }
    {
        /// kld 3D
    }
    {
        /// wheel of fortune
        auto ps_copy = ps;
        muse_amcl::WheelOfFortune m;
        m.apply(ps_copy);
        std::cout << "wheel of fortune particle set" << std::endl;
        for(auto &p : ps_copy.getParticles()) {
            std::cout << p.pose_.origin().x() << " " << p.weight_ << std::endl;
        }
        std::cout << "----------------" << std::endl;
    }




    return 0;
}


