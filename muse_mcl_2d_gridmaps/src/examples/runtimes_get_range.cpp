#include <muse_mcl_2d_gridmaps/maps/binary_gridmap.h>
#include <tf/tf.h>

int main(int argc, char *argv[])
{
    ros::Time::init();

    nav_msgs::OccupancyGrid grid;
    grid.header.stamp = ros::Time::now();
    grid.info.height = 2500;
    grid.info.width = 2500;
    grid.info.resolution = 0.05;
    grid.data.resize(2500 * 2500, 0);
    grid.info.origin.orientation = tf::createQuaternionMsgFromYaw(0.0);

    muse_mcl_2d_gridmaps::maps::BinaryGridMap::Ptr
            binary(new muse_mcl_2d_gridmaps::maps::BinaryGridMap(grid));


    muse_mcl_2d::Point2D start(62.5, 62.5);

    const double angle_incr = M_PI / 100.0;
    const double radius = 10.0;
    double angle = 0.0;
    double range = 0.0;
    muse_smc::Time now = muse_smc::Time::now();
    const std::size_t iterations = 100000;
    for(std::size_t i = 0 ; i < iterations ; ++i) {
        muse_mcl_2d::Point2D end = start + muse_mcl_2d::Vector2D(std::cos(angle) * radius,
                                                                 std::sin(angle) * radius);
        range = binary->getRange(start, end);
        angle += angle_incr;
    }
    std::cout << "took : " << (muse_smc::Time::now() - now).milliseconds() / iterations << std::endl;

    return 0;
}
