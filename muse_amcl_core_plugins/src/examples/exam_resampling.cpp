#include <muse_amcl/math/random.hpp>
#include <muse_amcl/particle_filter/particle_set.hpp>

#include "../resampling/multinomial.h"
#include "../resampling/stratified.h"
#include "../resampling/systematic.h"
#include "../resampling/wof.h"
#include "../resampling/residual.h"
#include "../resampling/kld_2d.h"
#include "../sampling/uniform_all_maps_2d.h"

#include <iostream>
#include <vector>

struct TestUniformAllMaps2D : public muse_mcl::UniformAllMaps2D
{
    virtual void apply(muse_mcl::Particle &particle) override
    {

    }
};

struct TestMultinomial : public muse_mcl::Multinomial
{
    TestMultinomial()
    {
        uniform_pose_sampler_.reset(new TestUniformAllMaps2D);
    }

};

struct TestStratified : public muse_mcl::Stratified
{
    TestStratified()
    {
        uniform_pose_sampler_.reset(new TestUniformAllMaps2D);
    }
};

struct TestSystematic : public muse_mcl::Systematic
{
    TestSystematic()
    {
        uniform_pose_sampler_.reset(new TestUniformAllMaps2D);
    }
};

struct TestResidual : public muse_mcl::Residual
{
    TestResidual()
    {
        uniform_pose_sampler_.reset(new TestUniformAllMaps2D);
    }
};

struct TestKLD2D : public muse_mcl::KLD2D
{
    TestKLD2D()
    {
        uniform_pose_sampler_.reset(new TestUniformAllMaps2D);
    }

};

int main(int argc, char *argv[])
{

    /// test random vecotor generation
    const std::size_t size = 10;
    {
        std::cout << "multimodal: " << std::endl;
        muse_mcl::math::random::Uniform<1> rng(0.0, 1.0);
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
        muse_mcl::math::random::Uniform<1> rng(0.0, 1.0);
        for(std::size_t i = 0 ; i < size ; ++i) {
            u[i] = (i + rng.get()) / size;
            std::cout << u[i] << " ";
        }
        std::cout << std::endl;
    }
    {
        std::cout << "systematic: " << std::endl;
        std::vector<double> u(size);
        muse_mcl::math::random::Uniform<1> rng(0.0, 1.0);
        double u_static = rng.get();
        for(std::size_t i = 0 ; i < size ; ++i) {
            u[i] = (i + u_static) / size;
            std::cout << u[i] << " ";
        }
        std::cout << std::endl;
    }

    /// test resampling
    muse_mcl::Indexation index({0.1, 0.1, 1./18. * M_PI});
    muse_mcl::ParticleSet set("frame", 10, index);
    {
        muse_mcl::Insertion insert = set.getInsertion();
        for(std::size_t i = 0 ; i < size ; ++i)
            insert.insert(muse_mcl::Particle());
        insert.close();

        /// random weights for the particles
        muse_mcl::math::random::Uniform<1> rng(0.0, 1.0);
        for(auto &w : set.getWeights()) {
            w = rng.get();
        }
        std::size_t i = 0;
        for(auto &p : set.getPoses()) {
            p.x() = (++i);
        }
    }
    {
        /// multinomial
        muse_mcl::ParticleSet ps_copy("frame", 10, index);
        auto insertion = ps_copy.getInsertion();
        for(const auto &p : set.getSamples())
            insertion.insert(p);
        insertion.close();

        muse_mcl::Multinomial m;
        m.apply(ps_copy);
        std::cout << "multinomial particle set" << std::endl;
        for(const auto &p : ps_copy.getSamples()) {
            std::cout << p.pose_.getOrigin().x() << " " << p.weight_ << std::endl;
        }
        std::cout << "----------------" << std::endl;
    }
    {
        /// stratified
        muse_mcl::ParticleSet ps_copy("frame", 10, index);
        auto insertion = ps_copy.getInsertion();
        for(const auto &p : set.getSamples())
            insertion.insert(p);
        insertion.close();

        muse_mcl::Stratified m;
        m.apply(ps_copy);
        std::cout << "stratified particle set" << std::endl;
        for(const auto &p : ps_copy.getSamples()) {
            std::cout << p.pose_.getOrigin().x() << " " << p.weight_ << std::endl;
        }
        std::cout << "----------------" << std::endl;
    }
    {
        /// systematic
        muse_mcl::ParticleSet ps_copy("frame", 10, index);
        auto insertion = ps_copy.getInsertion();
        for(const auto &p : set.getSamples())
            insertion.insert(p);
        insertion.close();

        muse_mcl::Systematic m;
        m.apply(ps_copy);
        std::cout << "systematic particle set" << std::endl;
        for(const auto &p : ps_copy.getSamples()) {
            std::cout << p.pose_.getOrigin().x() << " " << p.weight_ << std::endl;
        }
        std::cout << "----------------" << std::endl;
    }
    {
        /// residual
        muse_mcl::ParticleSet ps_copy("frame", 10, index);
        auto insertion = ps_copy.getInsertion();
        for(const auto &p : set.getSamples())
            insertion.insert(p);
        insertion.close();

        muse_mcl::Residual m;
        m.apply(ps_copy);
        std::cout << "residual particle set" << std::endl;
        for(const auto &p : ps_copy.getSamples()) {
            std::cout << p.pose_.getOrigin().x() << " " << p.weight_ << std::endl;
        }
        std::cout << "----------------" << std::endl;
    }
    {
        /// kld 2D
        /// residual
        muse_mcl::ParticleSet ps_copy("frame", 10, 20, index);
        auto insertion = ps_copy.getInsertion();
        for(const auto &p : set.getSamples())
            insertion.insert(p);
        insertion.close();

        TestKLD2D k;
        k.apply(ps_copy);
        std::cout << "kld2d particle set" << std::endl;
        for(const auto &p : ps_copy.getSamples()) {
            std::cout << p.pose_.getOrigin().x() << " " << p.weight_ << std::endl;
        }
        std::cout << "----------------" << std::endl;
    }
    {
        /// kld 3D
    }
    {
        /// wheel of fortune

        muse_mcl::ParticleSet ps_copy("frame", 10, index);
        auto insertion = ps_copy.getInsertion();
        for(const auto &p : set.getSamples())
            insertion.insert(p);
        insertion.close();

        muse_mcl::WheelOfFortune m;
        m.apply(ps_copy);
        std::cout << "wheel of fortune particle set" << std::endl;
        for(const auto &p : ps_copy.getSamples()) {
            std::cout << p.pose_.getOrigin().x() << " " << p.weight_ << std::endl;
        }
        std::cout << "----------------" << std::endl;
    }




    return 0;
}


