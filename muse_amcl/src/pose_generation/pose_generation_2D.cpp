#include <muse_amcl/pose_generation/pose_generation_2D.h>
#include <muse_amcl/utils/tf_eigen.hpp>
#include <muse_amcl/pose_generation/normal.hpp>
#include <muse_amcl/pose_generation/uniform.hpp>

using namespace muse_amcl;
using namespace pose_generation;

PoseGeneration2D::PoseGeneration2D(ParticleSet &particle_set) :
    PoseGeneration(particle_set)
{
}

void PoseGeneration2D::normal(const tf::Pose &pose,
                              const Covariance &pose_covariance)
{
    //// TODO : CHECK FOR MAP EXTENT

    std::vector<Map::ConstPtr> maps;
    for(auto m : map_providers_) {
        maps.emplace_back(m->map());
    }

    ParticleSet::Particles &particles = particle_set_.getParticles();
    const std::size_t sample_size = particle_set_.maximumSampleSize();
    particles.resize(sample_size);

    /// first check if mean is blocked
    auto valid = [maps](const tf::Pose &pose) {
        bool v = true;
        for(auto m : maps)
            v &= m->valid(pose);
        return v;
    };

    if(!valid(pose))
        throw std::runtime_error("Cannot initialize at given pose!");

    /// prepare the random generator and start generation

    using Metric = pose_generation::Metric;
    using Radian = pose_generation::Radian;
    Eigen::Vector3d mean = conversion::toEigen2D(pose);
    Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
    covariance.block<2,2>(0,0) = pose_covariance.block<2,2>(0,0);
    covariance(2,0) = pose_covariance(5,1);
    covariance(2,1) = pose_covariance(5,2);
    covariance(0,2) = pose_covariance(0,5);
    covariance(1,2) = pose_covariance(1,5);
    covariance(2,2) = pose_covariance(2,5);

    const double weight = 1.0 / sample_size;
    pose_generation::Normal<Metric,Metric,Radian> rng(mean, covariance);
    for(Particle &p : particles) {
        do {
            p.pose = conversion::toTF(rng());
            p.weight = weight;
        } while(!valid(p.pose));
    }
}

void PoseGeneration2D::uniform()
{
    //// TODO : CHECK FOR MAP EXTENT
    //// GET MIN / MAX

//    std::vector<Map::ConstPtr> maps;
//    for(auto m : map_providers_) {
//        maps.emplace_back(m->map());
//    }

//    ParticleSet::Particles &particles = particle_set_.getParticles();
//    const std::size_t sample_size = particle_set_.maximumSampleSize();
//    particles.resize(sample_size);

//    /// first check if mean is blocked
//    auto valid = [maps](const tf::Pose &pose) {
//        bool v = true;
//        for(auto m : maps)
//            v &= m->valid(pose);
//        return v;
//    };

//    /// prepare the random generator and start generation

//    using Metric = pose_generation::Metric;
//    using Radian = pose_generation::Radian;

//    const double weight = 1.0 / sample_size;
//    pose_generation::Normal<Metric,Metric,Radian> rng(mean, covariance);
//    for(Particle &p : particles) {
//        do {
//            p.pose = conversion::toTF(rng());
//            p.weight = weight;
//        } while(!valid(p.pose));
//    }

}
