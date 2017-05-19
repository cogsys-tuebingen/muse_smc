/// SYSTEM
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <opencv2/opencv.hpp>
#include <tf/tf.h>
#include <nav_msgs/GetMap.h>

/// PROJECT
#include <muse_mcl_core_plugins/maps_2d/binary_gridmap.h>

namespace muse_mcl {
class GridmapTestNode {
public:
    GridmapTestNode() :
        nh_private_("~"),
        has_pose_(false)
    {
        const std::string service_map  = nh_private_.param<std::string>("service_map", "/static_map");
        const std::string topic_pose   = nh_private_.param<std::string>("pose", "/initialpose");

        scale_              = nh_private_.param<double>("scale", 1.0);
        fov_                = nh_private_.param<double>("field_of_view", 270.0) * M_PI / 180.0;
        angular_resolution_ = nh_private_.param<double>("angular_resolution", 2.5) * M_PI / 180.0;
        maximum_range_      = nh_private_.param<double>("maximum_range", 30.0);
        pose_subscriber_    = nh_private_.subscribe(topic_pose, 1, &GridmapTestNode::pose, this);
        map_client_         = nh_private_.serviceClient<nav_msgs::GetMap>(service_map);

        nav_msgs::GetMap m;
        if(map_client_.call(m.request, m.response)) {
            gridmap_.reset(new maps::BinaryGridMap(m.response.map, 0.5));
            renderMap();
        } else {
            std::cerr << "Couldn't obtain map!" << std::endl;
            ros::shutdown();
        }

    }

    bool getRenderedMap(cv::Mat &rendered_map)
    {
        if(rendered_map_.empty())
            return false;

        rendered_map = rendered_map_.clone();
        if(has_pose_) {
            for(auto &d : scan_ray_directions_) {
                cv::line(rendered_map, pose_origin_, d, cv::Scalar(0,128,255), 1, CV_AA);
            }

            for(auto &p : scan_points_) {
                cv::circle(rendered_map, p, 2, cv::Scalar(0,0,255), CV_FILLED, CV_AA);
                cv::line(rendered_map, pose_origin_, p, cv::Scalar(0,0,255), 1, CV_AA);
            }


            cv::circle(rendered_map, pose_origin_, 5, cv::Scalar(255,0,0), 1, CV_AA);
            cv::line(rendered_map, pose_origin_, pose_direction_, cv::Scalar(0,255), 1, CV_AA);
        }
        cv::flip(rendered_map, rendered_map, 0);
        cv::resize(rendered_map, rendered_map, cv::Size(), scale_, scale_);
        return true;
    }

private:
    ros::NodeHandle                     nh_private_;
    ros::Subscriber                     map_subscriber_;
    ros::Subscriber                     pose_subscriber_;
    ros::ServiceClient                  map_client_;


    muse_mcl::maps::BinaryGridMap::Ptr gridmap_;

    cv::Mat                             rendered_map_;
    cv::Mat                             rendered_scan_;

    bool                                has_pose_;
    cv::Point                           pose_origin_;
    cv::Point                           pose_direction_;
    std::vector<cv::Point>              scan_points_;
    std::vector<cv::Point>              scan_ray_directions_;

    double                              scale_;
    double                              fov_;
    double                              maximum_range_;
    double                              angular_resolution_;

    void pose(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg)
    {
        if(gridmap_) {
            std::array<int, 2> index;
            math::Point position(msg->pose.pose.position.x, msg->pose.pose.position.y);
            if(gridmap_->toIndex(position, index)) {

                pose_origin_ = cv::Point(index[0], index[1]);

                const double yaw = tf::getYaw(msg->pose.pose.orientation);
                const int    len = 10; // px

                pose_direction_    = pose_origin_;
                pose_direction_.x += std::cos(yaw) * len;
                pose_direction_.y += std::sin(yaw) * len;

                /// lets generate a virtual scan
                scan_points_.clear();
                scan_ray_directions_.clear();

                const std::size_t rays = fov_ / angular_resolution_;
                double angle = yaw - 0.5 * fov_;
                for(std::size_t i = 0 ; i < rays ; ++i, angle += angular_resolution_) {
                    std::array<int, 2> scan_index;
                    math::Point scan_point = position;
                    scan_point.x() += std::cos(angle) * maximum_range_;
                    scan_point.y() += std::sin(angle) * maximum_range_;

                    gridmap_->toIndex(scan_point, scan_index);
                    scan_ray_directions_.emplace_back(cv::Point(scan_index[0], scan_index[1]));

                    auto it = gridmap_->getLineIterator(position, scan_point);
                    while(!it.done()) {
                        if(*it)
                            break;
                        rendered_map_.at<cv::Vec3b>(it.y(),it.x()) = cv::Vec3b(0,255,0);
                        ++it;
                    }

                    double range = gridmap_->getRange(position, scan_point);
                    if(range > 0.0) {
                        scan_point.x() = position.x() + std::cos(angle) * range;
                        scan_point.y() = position.y() + std::sin(angle) * range;
                        gridmap_->toIndex(scan_point, scan_index);
                        scan_points_.emplace_back(cv::Point(scan_index[0], scan_index[1]));
                    }
                }
                has_pose_ = true;
            }

            gridmap_->getRange(math::Point(), math::Point());

        }
    }

    void renderMap()
    {
        if(!gridmap_)
            return;

        const std::size_t cols = gridmap_->getWidth();
        const std::size_t rows = gridmap_->getHeight();

        rendered_map_ = cv::Mat(rows, cols, CV_8UC3, cv::Scalar());
        cv::Vec3b *render_map_ptr = rendered_map_.ptr<cv::Vec3b>();
        for(std::size_t i = 0 ; i < rows ; ++i) {
            for(std::size_t j = 0 ; j < cols ; ++j) {
                math::Point p;
                std::array<int, 2> index;
                gridmap_->fromIndex({static_cast<int>(j),
                                     static_cast<int>(i)},
                                    p);
                gridmap_->toIndex(p, index);

                if(gridmap_->at(index[0],index[1]) == 0) {
                    render_map_ptr[index[1] * cols + index[0]] = cv::Vec3b(255,255,255);
                }
            }
        }
    }
};
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "muse_amcl_test_gridmap");

    muse_mcl::GridmapTestNode gtn;

    while(ros::ok()) {
        ros::spinOnce();

        cv::Mat map;
        if(gtn.getRenderedMap(map)) {
            cv::imshow("map", map);
            int key = cv::waitKey(19) & 0xFF;
            if(key == 27)
                break;
        }
        ros::Rate(30.0).sleep();
    }
    return 0;
}
