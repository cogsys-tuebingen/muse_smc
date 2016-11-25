#include <muse_amcl/math/bounding_box.hpp>
#include <muse_amcl/math/angle.hpp>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <ros/ros.h>
#include <tf/tf.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "muse_amcl_exam_bounding_boxes");


    muse_amcl::math::Point min(-0.5,-0.5);
    muse_amcl::math::Point max(0.5,0.5,1.0);
    muse_amcl::math::BoundingBox bb(min, max);

    visualization_msgs::Marker      marker_template;
    marker_template.header.frame_id = "world";
    marker_template.header.stamp = ros::Time();
    marker_template.ns = "muse_amcl_exam_bounding_boxes";
    marker_template.id = 0;
    marker_template.type = visualization_msgs::Marker::LINE_LIST;
    marker_template.action = visualization_msgs::Marker::ADD;
    marker_template.scale.x = 0.025;
    marker_template.color.a = 1.0;
    marker_template.color.r = 0.0;
    marker_template.color.g = 1.0;
    marker_template.color.b = 0.0;


    auto emplace = [] (const std::size_t start,
            const std::size_t end,
            const muse_amcl::math::BoundingBox::Edges &edges,
            visualization_msgs::Marker &marker) {
        for(std::size_t i = start ; i < end; ++i) {
            geometry_msgs::Point p1;
            geometry_msgs::Point p2;
            p1.x = edges[i][0].x();
            p1.y = edges[i][0].y();
            p1.z = edges[i][0].z();
            p2.x = edges[i][1].x();
            p2.y = edges[i][1].y();
            p2.z = edges[i][1].z();
            marker.points.emplace_back(p1);
            marker.points.emplace_back(p2);
        }
    };
    ros::NodeHandle nh("~");
    ros::Publisher pub = nh.advertise<visualization_msgs::MarkerArray>("/markers", 1);


    double angle = 0.0;
    double angle_incr = 0.01;

    while(ros::ok()) {
        tf::Transform transform(tf::createQuaternionFromYaw(angle));

        visualization_msgs::MarkerArray markers;

        muse_amcl::math::BoundingBox be = bb * transform;
        muse_amcl::math::BoundingBox::Edges edges;
        be.edges(edges);

        visualization_msgs::Marker bottom = marker_template;
        emplace(0, 4, edges, bottom);
        markers.markers.emplace_back(bottom);

        visualization_msgs::Marker mid = marker_template;
        mid.color.r = 1.0;
        mid.id = 1;
        emplace(4, 8, edges, mid);
        markers.markers.emplace_back(mid);

        visualization_msgs::Marker top = marker_template;
        top.id = 2;
        top.color.r = 0.0;
        top.color.g = 0.0;
        top.color.b = 1.0;
        emplace(8, 12, edges, top);
        markers.markers.emplace_back(top);

        visualization_msgs::Marker corners = marker_template;
        corners.scale.x = 0.1;
        corners.scale.y = 0.1;
        corners.scale.z = 0.1;
        corners.id = 3;
        corners.type = visualization_msgs::Marker::SPHERE;
        corners.pose.position.x = be.minimum().x();
        corners.pose.position.y = be.minimum().y();
        corners.pose.position.z = be.minimum().z();
        corners.color.r = 0.f;
        corners.color.g = 0.f;
        corners.color.b = 0.f;
        markers.markers.emplace_back(corners);

        corners.id = 4;
        corners.pose.position.x = be.maximum().x();
        corners.pose.position.y = be.maximum().y();
        corners.pose.position.z = be.maximum().z();
        corners.color.r = 1.f;
        markers.markers.emplace_back(corners);

        pub.publish(markers);
        std::cout << angle << std::endl;

        angle = muse_amcl::math::angle::normalize(angle + angle_incr);
        ros::Rate(30).sleep();
    }



    return 0;
}
