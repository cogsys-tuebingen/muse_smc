#include <muse_mcl_2d/math/transform_2d.hpp>
#include "transform_2d_legacy.hpp"
#include <tf/tf.h>
#include <muse_smc/time/time.hpp>

const std::size_t ITERATIONS = 1000000;

void constructors()
{
    muse_smc::Time start = muse_smc::Time::now();
    double yaw = 0.0;
    for(std::size_t i = 0 ; i < ITERATIONS ; ++i) {
        muse_mcl_2d::Transform2D t;
        yaw = t.yaw();
    }
    std::cout << "empty:" << std::endl;
    std::cout << "took time: " << (muse_smc::Time::now() - start).milliseconds() << "ms" << std::endl;

    start = muse_smc::Time::now();
    for(std::size_t i = 0 ; i < ITERATIONS ; ++i) {
        muse_mcl_2d::Transform2D t(i, i);
        yaw = t.yaw();
    }
    std::cout << "x y:" << std::endl;
    std::cout << "took time: " << (muse_smc::Time::now() - start).milliseconds() << "ms" << std::endl;

    start = muse_smc::Time::now();
    muse_mcl_2d::Vector2D v(0.0,0.0);
    for(std::size_t i = 0 ; i < ITERATIONS ; ++i) {
        muse_mcl_2d::Transform2D t(v);
        yaw = t.yaw();
        v.x() += i;
    }
    std::cout << "v:" << std::endl;
    std::cout << "took time: " << (muse_smc::Time::now() - start).milliseconds() << "ms" << std::endl;

    start = muse_smc::Time::now();
    for(std::size_t i = 0 ; i < ITERATIONS ; ++i) {
        muse_mcl_2d::Transform2D t(i,i,i);
        yaw = t.yaw();
    }
    std::cout << "x y yaw:" << std::endl;
    std::cout << "took time: " << (muse_smc::Time::now() - start).milliseconds() << "ms" << std::endl;

    start = muse_smc::Time::now();
    for(std::size_t i = 0 ; i < ITERATIONS ; ++i) {
        muse_mcl_2d::Transform2D t(v,i);
        yaw = t.yaw();
    }
    std::cout << "v yaw:" << std::endl;
    std::cout << "took time: " << (muse_smc::Time::now() - start).milliseconds() << "ms" << std::endl;

    start = muse_smc::Time::now();
    muse_mcl_2d::Transform2D t(0.0,0.0);
    for(std::size_t i = 0 ; i < ITERATIONS ; ++i) {
        muse_mcl_2d::Transform2D t_(t);
        yaw = t_.yaw();
    }
    std::cout << "t:" << std::endl;
    std::cout << "took time: " << (muse_smc::Time::now() - start).milliseconds() << "ms" << std::endl;
}


int main(int argc, char *argv[])
{
    constructors();
    return 0;
}
