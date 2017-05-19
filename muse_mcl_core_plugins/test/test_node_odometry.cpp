#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>

nav_msgs::OdometryConstPtr odom_last;

void callback(const nav_msgs::OdometryConstPtr &msg)
{
    auto delta = [](const nav_msgs::OdometryConstPtr &start,
                   const nav_msgs::OdometryConstPtr &end)
    {

        tf::Pose start_pose;
        tf::poseMsgToTF(start->pose.pose, start_pose);
        tf::Pose end_pose;
        tf::poseMsgToTF(end->pose.pose, end_pose);

        auto d = start_pose * end_pose.inverse();
        std::cout << std::hypot(start->pose.pose.position.x - end->pose.pose.position.x,
                                start->pose.pose.position.y - end->pose.pose.position.y)
                  << std::endl;
        std::cout << d.getOrigin().length() << " " << d.getRotation().getAngle() << std::endl;;

    };

    if(odom_last) {
        delta(odom_last, msg);
    }
    odom_last = msg;
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "test_node_odometry");
    ros::NodeHandle nh("~");
    ros::Subscriber sub_odom = nh.subscribe("/odom", 1, callback);

    ros::spin();

    return 0;
}
