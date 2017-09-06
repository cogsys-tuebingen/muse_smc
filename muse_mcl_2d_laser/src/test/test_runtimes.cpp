#include <muse_mcl_2d_laser/laser/laser_2d_scan.hpp>

#include <muse_mcl_2d/math/transform_2d.hpp>
#include <muse_mcl_2d/math/covariance_2d.hpp>

int main(int argc, char *argv[])
{

    muse_mcl_2d_laser::LaserScan2D::Rays rays;
    for(std::size_t i = 0 ; i < 1000 ; ++i) {
        rays.push_back(muse_mcl_2d_laser::LaserScan2D::Ray());
    }

    muse_smc::Time t = muse_smc::Time::now();
    double x,y;
    for(auto &r : rays) {
        x = r.point.x();
        y = r.point.y();
    }
    std::cout << (muse_smc::Time::now() - t).milliseconds() << std::endl;
    std::cout << sizeof(muse_mcl_2d_laser::LaserScan2D::Ray) << std::endl;
    std::cout << sizeof(muse_mcl_2d::Transform2D) << std::endl;
    std::cout << sizeof(muse_mcl_2d::Covariance2D) << std::endl;

    return 0;
}
