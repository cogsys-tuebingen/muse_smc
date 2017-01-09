#include <muse_amcl/math/bounding_box.hpp>
#include <muse_amcl/math/bounding_rectangle.hpp>
#include <muse_amcl/math/angle.hpp>
#include <muse_amcl/pose_generators/uniform.hpp>

#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <ros/ros.h>
#include <tf/tf.h>

#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/geometries/point_xy.hpp>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "muse_amcl_exam_bounding_boxes");

    using Point   = boost::geometry::model::point<double, 3, boost::geometry::cs::cartesian>;
    using Polygon = boost::geometry::model::polygon<Point>;

    Polygon polygon_2d_rotated;
    Polygon polygon_2d_still;

    polygon_2d_rotated.outer().emplace_back(Point(-.5,-.5,.0));
    polygon_2d_rotated.outer().emplace_back(Point(-.5, .5,.0));
    polygon_2d_rotated.outer().emplace_back(Point( .5, .5,.0));
    polygon_2d_rotated.outer().emplace_back(Point( .5,-.5,.0));
    polygon_2d_still.outer() = polygon_2d_rotated.outer();

    visualization_msgs::Marker  marker_template;
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


    ros::NodeHandle nh("~");
    ros::Publisher pub_boxes = nh.advertise<visualization_msgs::MarkerArray>("/markers", 1);
    //    ros::Publisher pub_poses = nh.advertise<geometry_msgs::PoseArray>("/poses", 1);


    auto toGeometryPoint = [](const Point &p)
    {
        geometry_msgs::Point g;
        g.x = p.get<0>();
        g.y = p.get<1>();
        g.z = p.get<2>();
        return g;
    };

    auto toTFPoint = [] (const Point &p)
    {
        return tf::Point(p.get<0>(), p.get<1>(), p.get<2>());
    };

    auto fromTFPoint = [] (const tf::Point &p)
    {
        return Point(p.x(), p.y(), p.z());
    };

    auto toMarker = [toGeometryPoint] (const Polygon &polygon,
                        visualization_msgs::Marker &marker) {
        marker.type = visualization_msgs::Marker::LINE_STRIP;
        for(const auto p : polygon.outer()) {
            marker.points.emplace_back(toGeometryPoint(p));
        }
        marker.points.emplace_back(toGeometryPoint(polygon.outer().front()));
    };

    auto transform = [toTFPoint, fromTFPoint] (const Polygon &polygon,
                                               const tf::Transform &transform)
    {
        Polygon transformed;
        for(const auto p : polygon.outer()) {
            tf::Point trasnformed_point = transform * toTFPoint(p);
            transformed.outer().emplace_back(fromTFPoint(trasnformed_point));
        }
        return transformed;
    };


    double yaw = muse_amcl::math::angle::toRad(1.);
    tf::Transform rotation(tf::createQuaternionFromYaw(yaw));
    while(ros::ok()) {
        int id = 0;

        visualization_msgs::MarkerArray markers;
        /// static polygon
        visualization_msgs::Marker marker_polygon_2d_still = marker_template;
        marker_polygon_2d_still.id = ++id;
        toMarker(polygon_2d_still, marker_polygon_2d_still);
        markers.markers.emplace_back(marker_polygon_2d_still);
        /// rotating polygon
        visualization_msgs::Marker marker_polygon_2d_rotating = marker_template;
        marker_polygon_2d_rotating.id = ++id;
        polygon_2d_rotated = transform(polygon_2d_rotated, rotation);
        toMarker(polygon_2d_rotated, marker_polygon_2d_rotating);
        markers.markers.emplace_back(marker_polygon_2d_rotating);

        /// put out and sleep
        pub_boxes.publish(markers);
        ros::Rate(30).sleep();
    }



    return 0;
}
