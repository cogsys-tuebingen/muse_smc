/// SYSTEM
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <opencv2/opencv.hpp>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <nav_msgs/GetMap.h>
#include <mutex>
#include <thread>
#include <atomic>


/// PROJECT
#include <muse_amcl_core_plugins/maps_2d/binary_gridmap.h>
#include <muse_amcl/ParticleSetMsg.h>

namespace muse_amcl {
class GridmapTestNode {
public:
    GridmapTestNode() :
        nh_private_("~"),
        has_poses_(false)
    {
        const std::string service_map  = nh_private_.param<std::string>("service_map", "/static_map");
        const std::string topic_particles   = nh_private_.param<std::string>("topic", "/muse_amcl/particles");

        scale_              = nh_private_.param<double>("scale", 0.5);
        fov_                = nh_private_.param<double>("field_of_view", 270.0) * M_PI / 180.0;
        angular_resolution_ = nh_private_.param<double>("angular_resolution", 2.5) * M_PI / 180.0;
        maximum_range_      = nh_private_.param<double>("maximum_range", 30.0);
        pose_subscriber_    = nh_private_.subscribe(topic_particles, 1, &GridmapTestNode::particles, this);
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
        if(has_poses_) {
            const std::size_t sample_size = pose_origins_.size();
            for(std::size_t i = 0 ; i < sample_size ; ++i) {
                auto &scan_points = scan_points_[i];
                auto &pose_origin = pose_origins_[i];

                for(auto &p : scan_points) {
                    cv::circle(rendered_map, p, 2, cv::Scalar(0,0,255,255), CV_FILLED, CV_AA);
                    cv::line(rendered_map, pose_origin, p, cv::Scalar(0,0,255,5), 1, CV_AA);
                }

            }
            for(std::size_t i = 0 ; i < sample_size ; ++i) {
                auto &pose_origin = pose_origins_[i];
                cv::circle(rendered_map, pose_origin, 5, cv::Scalar(255,0,0, 255), 1, CV_AA);
                cv::line(rendered_map, pose_origin, pose_directions_[i], cv::Scalar(0,255, 255), 1, CV_AA);
            }
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
    tf::TransformListener               tf_;

    muse_amcl::maps::BinaryGridMap::Ptr gridmap_;

    cv::Mat                             rendered_map_;
    cv::Mat                             rendered_scan_;


    bool                                has_poses_;
    std::vector<cv::Point>              pose_origins_;
    std::vector<cv::Point>              pose_directions_;
    std::vector<std::vector<cv::Point>> scan_points_;

    double                              scale_;
    double                              fov_;
    double                              maximum_range_;
    double                              angular_resolution_;

    void particles(const muse_amcl::ParticleSetMsg::ConstPtr &particles)
    {
        if(gridmap_) {
            std::array<int, 2> index;

            scan_points_.clear();
            pose_origins_.clear();
            pose_directions_.clear();

            //// lets get the transformations
            tf::StampedTransform base_T_laser;
            if(!tf_.waitForTransform("/base_link", "/sick_front", particles->header.stamp, ros::Duration(0.1)))
                return;
            tf_.lookupTransform("/base_link", "/sick_front",particles->header.stamp, base_T_laser);
            tf::StampedTransform map_T_world;
            if(!tf_.waitForTransform("/world", "/map", particles->header.stamp, ros::Duration(0.1)))
                return;
            tf_.lookupTransform("/world", "/map",particles->header.stamp, map_T_world);

            for(const muse_amcl::ParticleMsg &p : particles->particles) {
                tf::Pose      pose;
                tf::poseMsgToTF(p.pose, pose);

                std::cout << p.weight << std::endl;

                pose = static_cast<tf::Transform>(map_T_world) *
                       pose *
                       static_cast<tf::Transform>(base_T_laser);

                math::Point position(pose.getOrigin().x(), pose.getOrigin().y());


                if(gridmap_->toIndex(position, index)) {

                    pose_origins_.emplace_back(cv::Point(index[0], index[1]));

                    const double yaw = tf::getYaw(pose.getRotation());
                    const int    len = 10; // px

                    pose_directions_.emplace_back(pose_origins_.back());
                    pose_directions_.back().x += std::cos(yaw) * len;
                    pose_directions_.back().y += std::sin(yaw) * len;

                    /// lets generate a virtual scan
                    std::vector<cv::Point> scan_points;

                    const std::size_t rays = fov_ / angular_resolution_;
                    double angle = yaw - 0.5 * fov_;
                    for(std::size_t i = 0 ; i < rays ; ++i, angle += angular_resolution_) {
                        std::array<int, 2> scan_index;
                        math::Point scan_point = position;
                        scan_point.x() += std::cos(angle) * maximum_range_;
                        scan_point.y() += std::sin(angle) * maximum_range_;

                        gridmap_->toIndex(scan_point, scan_index);

                        double range = gridmap_->getRange(position, scan_point);
                        if(range > 0.0) {
                            scan_point.x() = position.x() + std::cos(angle) * range;
                            scan_point.y() = position.y() + std::sin(angle) * range;
                            gridmap_->toIndex(scan_point, scan_index);
                            scan_points.emplace_back(cv::Point(scan_index[0], scan_index[1]));
                        }
                    }
                    scan_points_.emplace_back(scan_points);
                }
            }
            has_poses_ = true;
        }



    }




    void renderMap()
    {
        if(!gridmap_)
            return;

        const std::size_t cols = gridmap_->getWidth();
        const std::size_t rows = gridmap_->getHeight();

        rendered_map_ = cv::Mat(rows, cols, CV_8UC4, cv::Scalar());
        cv::Vec4b *render_map_ptr = rendered_map_.ptr<cv::Vec4b>();
        for(std::size_t i = 0 ; i < rows ; ++i) {
            for(std::size_t j = 0 ; j < cols ; ++j) {
                math::Point p;
                std::array<int, 2> index;
                gridmap_->fromIndex({static_cast<int>(j),
                                     static_cast<int>(i)},
                                    p);
                gridmap_->toIndex(p, index);

                if(gridmap_->at(index[0],index[1]) == 0) {
                    render_map_ptr[index[1] * cols + index[0]] = cv::Vec4b(255,255,255);
                }
            }
        }
    }
};

class Window {
public:
    Window(const std::string &window_name) :
        window_name_(window_name),
        running_(false),
        stop_(false)
    {
        worker_thread_ = std::thread([this]{loop();});
        worker_thread_.detach();
    }

    virtual ~Window()
    {
        if(!running_)
            return;

        stop_ = true;
        if(worker_thread_.joinable())
            worker_thread_.join();
    }

    void setContent(const cv::Mat &mat)
    {
        std::unique_lock<std::mutex>(content_mutex_);
        content_ = mat.clone();
    }

    bool isRunnning() const
    {
        return running_;
    }

private:
    const std::string                       window_name_;

    std::atomic_bool                        running_;
    std::atomic_bool                        stop_;
    std::thread                             worker_thread_;

    std::mutex                              content_mutex_;
    cv::Mat                                 content_;

    inline void loop()
    {
        running_ = true;
        while(!stop_) {
            cv::namedWindow(window_name_);

            std::unique_lock<std::mutex>(content_mutex_);
            cv::imshow(window_name_, content_);
            cv::waitKey(19);
            int key = cv::waitKey(19) & 0xFF;
            if(key == 27)
                break;
        }
        running_ = false;
        cv::destroyAllWindows();
    }

};

}




int main(int argc, char *argv[])
{
    ros::init(argc, argv, "muse_amcl_test_gridmap");

    muse_amcl::GridmapTestNode gtn;
    muse_amcl::Window window_map("map");
  //  muse_amcl::Window window_weights("weights");
    while(ros::ok()) {
        ros::spinOnce();

        cv::Mat map;
        if(gtn.getRenderedMap(map)) {
            window_map.setContent(map);
        }
        ros::Rate(30.0).sleep();
    }
    return 0;
}
