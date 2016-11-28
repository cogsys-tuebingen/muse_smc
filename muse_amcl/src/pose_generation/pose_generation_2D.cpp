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

void PoseGeneration2D::normal(const math::Pose       &pose,
                              const math::Covariance &pose_covariance)
{
//    std::vector<Map::ConstPtr> maps;
//    for(auto m : map_providers_) {
//        maps.emplace_back(m->map());
//    }

//    ParticleSet::Particles &particles = particle_set_.getParticles();
//    const std::size_t sample_size = particle_set_.maximumSampleSize();
//    particles.resize(sample_size);

//    /// first check if mean is blocked
//    auto valid = [maps](const math::Pose &pose) {
//        bool v = true;
//        for(auto m : maps)
//            v &= m->valid(pose);
//        return v;
//    };

//    if(!valid(pose))
//        throw std::runtime_error("Cannot initialize at given pose!");

//    /// prepare the random generator and start generation

//    using Metric = pose_generation::Metric;
//    using Radian = pose_generation::Radian;
//    Eigen::Vector3d mean = pose.eigen3D();
//    Eigen::Matrix3d covariance = pose_covariance.eigen3D();

//    const double weight = 1.0 / sample_size;
//    pose_generation::Normal<Metric,Metric,Radian> rng(mean, covariance);
//    for(Particle &p : particles) {
//        do {
//            p.pose = conversion::toTF(rng());
//            p.weight = weight;
//        } while(!valid(p.pose));
//    }
}

void PoseGeneration2D::uniform()
{
    /**
      +-----------------------------------------+
      | Rejection Sampling                      |
      | -> Major Map                            |
      |    + generate within major map          |
      |    + check with minor maps              |
      | -> Minimum Bounding Volume ( Interval ) |
      | -> Maximum Bounding Volume ( Interval ) |
      | -> Optimization                         |
      |    + Map Free Space Definition          |
      |    + and sampling.                      |
      +-----------------------------------------+

     **/


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

void PoseGeneration2D::doSetup(ros::NodeHandle &nh_private)
{

}
