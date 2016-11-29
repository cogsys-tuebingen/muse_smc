#include <muse_amcl/math/bounding_box.hpp>
#include <muse_amcl/math/bounding_rectangle.hpp>
#include <muse_amcl/math/angle.hpp>
#include <muse_amcl/pose_generators/uniform.hpp>

#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <ros/ros.h>
#include <tf/tf.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "muse_amcl_exam_bounding_boxes");


    muse_amcl::math::Point min_bb1(-0.5,-0.5);
    muse_amcl::math::Point max_bb1(0.5,0.5,1.0);
    muse_amcl::math::Point min_bb2(-1.5,-1.5);
    muse_amcl::math::Point max_bb2(0.0,0.0,1.0);
    muse_amcl::math::Point min_br(-0.5,-0.5, 0.0);
    muse_amcl::math::Point max_br(0.5,0.5, 0.0);
    muse_amcl::math::BoundingBox bb1(min_bb1, max_bb1);
    muse_amcl::math::BoundingBox bb2(min_bb2, max_bb2);
    muse_amcl::math::BoundingRectangle br(min_br, max_br);

    muse_amcl::math::Point min_br1(1, 1, 0.0);
    muse_amcl::math::Point max_br1(2, 2, 0.0);
    muse_amcl::math::Point min_br2(2.5,2.5, 0.0);
    muse_amcl::math::Point max_br2(3.5,3.5, 0.0);
    muse_amcl::math::BoundingRectangle br1(min_br1, max_br1);
    muse_amcl::math::BoundingRectangle br2(min_br2, max_br2);

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


    auto emplace3D = [] (const std::size_t start,
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

    auto emplace2D = [] (const std::size_t start,
            const std::size_t end,
            const muse_amcl::math::BoundingRectangle::Edges &edges,
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
    ros::Publisher pub_boxes = nh.advertise<visualization_msgs::MarkerArray>("/markers", 1);
    ros::Publisher pub_poses = nh.advertise<geometry_msgs::PoseArray>("/poses", 1);

    double yaw = 0.0;
    double yaw_incr = 0.01;
    double pitch = 0.0;
    double pitch_incr = 0.005;
    double roll = 0.0;
    double roll_incr = 0.0;


    while(ros::ok()) {
        int id = 0;
        tf::Transform translation_bb(tf::createIdentityQuaternion(), tf::Vector3(1.0, 0.0, 0.0));
        tf::Transform translation_br(tf::createIdentityQuaternion(), tf::Vector3(-1.0, 0.0, 0.0));
        tf::Transform rotation(tf::createQuaternionFromRPY(roll, pitch, yaw));

        yaw = muse_amcl::math::angle::normalize(yaw + yaw_incr);
        roll = muse_amcl::math::angle::normalize(roll + roll_incr);
        pitch = muse_amcl::math::angle::normalize(pitch + pitch_incr);

        visualization_msgs::MarkerArray markers;

        // bounding box itself
        muse_amcl::math::BoundingBox       bb_transformed = (translation_bb * rotation) * bb1;
        muse_amcl::math::BoundingRectangle br_transformed = (translation_br * rotation) * br;

        muse_amcl::math::BoundingBox::Edges be_edges;
        bb_transformed.edges(be_edges);
        muse_amcl::math::BoundingRectangle::Edges br_edges;
        br_transformed.edges(br_edges);

        visualization_msgs::Marker bb_bottom = marker_template;
        bb_bottom.id = ++id;
        emplace3D(0, 4, be_edges, bb_bottom);
        markers.markers.emplace_back(bb_bottom);

        visualization_msgs::Marker bb_mid = marker_template;
        bb_mid.id = ++id;
        emplace3D(4, 8, be_edges, bb_mid);
        markers.markers.emplace_back(bb_mid);

        visualization_msgs::Marker bb_top = marker_template;
        bb_top.id = ++id;
        emplace3D(8, 12, be_edges, bb_top);
        markers.markers.emplace_back(bb_top);

        visualization_msgs::Marker br_bottom = marker_template;
        br_bottom.id = ++id;
        emplace2D(0, 4, br_edges, br_bottom);
        markers.markers.emplace_back(br_bottom);

        // axis aligned
        muse_amcl::math::BoundingBox bba = bb_transformed.axisAlignedEnclosing();
        muse_amcl::math::BoundingBox::Edges bba_edges = bba.edges();
        bb_bottom.color.g = 0.0;
        bb_bottom.color.b = 1.0;
        bb_bottom.id = ++id;
        bb_bottom.points.clear();
        emplace3D(0, 12, bba_edges, bb_bottom);
        markers.markers.emplace_back(bb_bottom);

        muse_amcl::math::BoundingRectangle bar = br_transformed.axisAlignedEnclosingXY();
        muse_amcl::math::BoundingRectangle::Edges bar_edges = bar.edges();
        br_bottom.id = ++id;
        br_bottom.color.g = 0.0;
        br_bottom.color.b = 1.0;
        br_bottom.points.clear();
        emplace2D(0,4, bar_edges, br_bottom);
        markers.markers.emplace_back(br_bottom);

        bar = br_transformed.axisAlignedEnclosingXZ();
        bar_edges = bar.edges();
        br_bottom.id = ++id;
        br_bottom.color.g = 1.0;
        br_bottom.color.b = 1.0;
        br_bottom.points.clear();
        emplace2D(0,4, bar_edges, br_bottom);
        markers.markers.emplace_back(br_bottom);

        bar = br_transformed.axisAlignedEnclosingYZ();
        bar_edges = bar.edges();
        br_bottom.id = ++id;
        br_bottom.color.b = 0.0;
        br_bottom.color.g = 1.0;
        br_bottom.color.r = 1.0;
        br_bottom.points.clear();
        emplace2D(0,4, bar_edges, br_bottom);
        markers.markers.emplace_back(br_bottom);

        visualization_msgs::Marker brim = marker_template;
        brim.id = ++id;
        emplace2D(0,4,br1.edges(), brim);
        markers.markers.emplace_back(brim);

        brim.id = ++id;
        brim.points.clear();
        emplace2D(0,4,br2.edges(), brim);
        markers.markers.emplace_back(brim);

        muse_amcl::math::BoundingRectangle bri;
        brim.id = ++id;
        brim.points.clear();
        brim.color.b = 1.0;
        if(br2.axisAlignedIntersection(br1, bri))
        {
            emplace2D(0, 4, bri.edges(), brim);
            markers.markers.emplace_back(brim);
        }
        muse_amcl::math::BoundingRectangle brum;
        brim.id = ++id;
        brim.points.clear();
        brim.color.b = 1.0;
        brim.color.r = 1.0;
        br2.axisAlignedUnion(br1, brum);
        emplace2D(0, 4, brum.edges(), brim);
        markers.markers.emplace_back(brim);

        muse_amcl::math::BoundingBox bbum;
        brim.id = ++id;
        brim.points.clear();
        brim.color.r = 1.0;
        brim.color.g = 1.0;
        brim.color.b = 0.0;
        if(bb2.axisAlignedIntersection(bb_transformed, bbum)) {
            emplace3D(0, 12, bbum.edges(), brim);
            markers.markers.emplace_back(brim);
        }

        // corner points
        visualization_msgs::Marker corners = marker_template;
        corners.scale.x = 0.1;
        corners.scale.y = 0.1;
        corners.scale.z = 0.1;
        corners.id = ++id;
        corners.type = visualization_msgs::Marker::SPHERE;
        corners.pose.position.x = bb_transformed.minimum().x();
        corners.pose.position.y = bb_transformed.minimum().y();
        corners.pose.position.z = bb_transformed.minimum().z();
        corners.color.r = 0.f;
        corners.color.g = 0.f;
        corners.color.b = 0.f;
        markers.markers.emplace_back(corners);

        corners.id = ++id;
        corners.pose.position.x = bb_transformed.maximum().x();
        corners.pose.position.y = bb_transformed.maximum().y();
        corners.pose.position.z = bb_transformed.maximum().z();
        corners.color.r = 1.f;
        markers.markers.emplace_back(corners);

        corners.id = ++id;
        corners.pose.position.x = br_transformed.minimum().x();
        corners.pose.position.y = br_transformed.minimum().y();
        corners.pose.position.z = br_transformed.minimum().z();
        corners.color.r = 0.f;
        markers.markers.emplace_back(corners);

        corners.id = ++id;
        corners.pose.position.x = br_transformed.maximum().x();
        corners.pose.position.y = br_transformed.maximum().y();
        corners.pose.position.z = br_transformed.maximum().z();
        corners.color.r = 1.f;
        markers.markers.emplace_back(corners);

        muse_amcl::math::random::Uniform<6>::Vector min;
        muse_amcl::math::random::Uniform<6>::Vector max;
        min(0) = bba.minimum().x();
        min(1) = bba.minimum().y();
        min(2) = bba.minimum().z();
        min(3) = 0.0;
        min(4) = 0.0;
        min(5) = 0.0;
        max(0) = bba.maximum().x();
        max(1) = bba.maximum().y();
        max(2) = bba.maximum().z();
        max(3) = 2 * M_PI;
        max(4) = 2 * M_PI;
        max(5) = 2 * M_PI;

        using M = muse_amcl::pose_generation::Metric;
        using R = muse_amcl::pose_generation::Metric;
        using RNG = muse_amcl::pose_generation::Uniform<M, M, M, R, R, R>;

        RNG rng(min, max);
        geometry_msgs::PoseArray pose_arr;
        pose_arr.header.frame_id = marker_template.header.frame_id;
        for(std::size_t i = 0 ; i < 1000 ; ++i) {
            geometry_msgs::Pose p;
            RNG::Vector v;
            do {
                v = rng();
            } while(!bb_transformed.contains(muse_amcl::math::Point(v(0),v(1),v(2))));
            p.position.x = v(0);
            p.position.y = v(1);
            p.position.z = v(2);
            p.orientation = tf::createQuaternionMsgFromRollPitchYaw(v(3), v(4), v(5));
            pose_arr.poses.emplace_back(p);
        }

        pub_boxes.publish(markers);
        pub_poses.publish(pose_arr);

        ros::Rate(100).sleep();
    }



    return 0;
}
