#include <gtest/gtest.h>

#include <muse_amcl/math/bounding_rectangle.hpp>
#include <tf/tf.h>

using Point = muse_amcl::math::Point;
using Points = std::array<Point,4> ;
using Edge   = std::array<Point,2>;
using Edges  = std::array<Edge, 4>;
using BoundingRectangle = muse_amcl::math::BoundingRectangle;


TEST(boundingRectangle, constructorMinMax)
{
    Point min;
    Point max = Point(1,1);

    Points expected_corners =
    {
        min,
        Point(max.x(), min.y()),
        max,
        Point(min.x(), max.y())
    };

    BoundingRectangle r(min, max);

    EXPECT_EQ(min, r.minimum());
    EXPECT_EQ(max, r.maximum());

    auto corners = r.corners();
    for(std::size_t i = 0 ; i < 4 ; ++i) {
        EXPECT_EQ(expected_corners[i], corners[i]);
    }
}

TEST(boundingRectangle, constructorMinAxis)
{
    {
        Point min;
        Point max = Point(1.,1.);

        Points expected_corners =
        {
            min,
            Point(max.x(), min.y()),
            max,
            Point(min.x(), max.y())
        };

        BoundingRectangle r(min, tf::Vector3(1.0, 0.0, .0), tf::Vector3(0.0, 1.0, .0));

        EXPECT_EQ(min.x(), r.minimum().x());
        EXPECT_EQ(min.y(), r.minimum().y());
        EXPECT_EQ(max.x(), r.maximum().x());
        EXPECT_EQ(max.y(), r.maximum().y());

        auto corners = r.corners();
        for(std::size_t i = 0 ; i < 4 ; ++i) {
            EXPECT_EQ(expected_corners[i].x(), corners[i].x());
            EXPECT_EQ(expected_corners[i].y(), corners[i].y());
        }
    }
    {
        Point min;
        Point max = Point(0.,2.);

        Points expected_corners =
        {
            min,
            Point(1, 1),
            max,
            Point(-1,1)
        };

        BoundingRectangle r(min, tf::Vector3(1.0, 1.0, .0), tf::Vector3(-1.0, 1.0, .0));

        EXPECT_EQ(min.x(), r.minimum().x());
        EXPECT_EQ(min.y(), r.minimum().y());
        EXPECT_EQ(max.x(), r.maximum().x());
        EXPECT_EQ(max.y(), r.maximum().y());

        auto corners = r.corners();
        for(std::size_t i = 0 ; i < 4 ; ++i) {
            EXPECT_EQ(expected_corners[i].x(), corners[i].x());
            EXPECT_EQ(expected_corners[i].y(), corners[i].y());
        }
    }
}

TEST(boundingRectangle, edges)
{
    {
        Point min;
        Point max = Point(1.,1.);
        Points expected_corners =
        {
            min,
            Point(max.x(), min.y()),
            max,
            Point(min.x(), max.y())
        };
        Edges expected_edges;
        expected_edges[0][0] = expected_corners[0];
        expected_edges[0][1] = expected_corners[1];
        expected_edges[1][0] = expected_corners[1];
        expected_edges[1][1] = expected_corners[2];
        expected_edges[2][0] = expected_corners[2];
        expected_edges[2][1] = expected_corners[3];
        expected_edges[3][0] = expected_corners[3];
        expected_edges[3][1] = expected_corners[0];

        BoundingRectangle r(min, tf::Vector3(1.0, 0.0, .0), tf::Vector3(0.0, 1.0, .0));

        Edges edges;
        r.edges(edges);
        for(std::size_t i = 0 ; i < 4 ; ++i) {
            EXPECT_EQ(expected_edges[i][0].x(), edges[i][0].x());
            EXPECT_EQ(expected_edges[i][0].y(), edges[i][0].y());
            EXPECT_EQ(expected_edges[i][1].x(), edges[i][1].x());
            EXPECT_EQ(expected_edges[i][1].y(), edges[i][1].y());
        }
    }
    {
        Point min;
        Point max = Point(0.,2.);

        Points expected_corners =
        {
            min,
            Point(1, 1),
            max,
            Point(-1,1)
        };

        BoundingRectangle r(min, tf::Vector3(1.0, 1.0, .0), tf::Vector3(-1.0, 1.0, .0));

        Edges expected_edges;
        expected_edges[0][0] = expected_corners[0];
        expected_edges[0][1] = expected_corners[1];
        expected_edges[1][0] = expected_corners[1];
        expected_edges[1][1] = expected_corners[2];
        expected_edges[2][0] = expected_corners[2];
        expected_edges[2][1] = expected_corners[3];
        expected_edges[3][0] = expected_corners[3];
        expected_edges[3][1] = expected_corners[0];

        auto edges = r.edges();
        for(std::size_t i = 0 ; i < 4 ; ++i) {
            EXPECT_EQ(expected_edges[i][0].x(), edges[i][0].x());
            EXPECT_EQ(expected_edges[i][0].y(), edges[i][0].y());
            EXPECT_EQ(expected_edges[i][1].x(), edges[i][1].x());
            EXPECT_EQ(expected_edges[i][1].y(), edges[i][1].y());
        }
    }
}

TEST(boundingRectangle, transform)
{
    Point min;

    tf::Transform rot(tf::createQuaternionFromYaw(M_PI / 4.0));
    tf::Transform trans(tf::createIdentityQuaternion(),
                        tf::Vector3(2.0, -1.0, 0.0));

    BoundingRectangle r(min, tf::Vector3(1.0, 0.0, .0), tf::Vector3(0.0, 1.0, .0));
    {
        Point min;
        Point max = Point(0.,std::sqrt(2.));

        Points expected_corners =
        {
            min,
            Point(0.5 * std::sqrt(2.), 0.5 * std::sqrt(2.)),
            max,
            Point(-0.5 * std::sqrt(2.),0.5 * std::sqrt(2.))
        };

        BoundingRectangle rotated = r;
        rotated.transform(rot);

        EXPECT_NEAR(min.x(), rotated.minimum().x(), 1e-6);
        EXPECT_NEAR(min.y(), rotated.minimum().y(), 1e-6);
        EXPECT_NEAR(max.x(), rotated.maximum().x(), 1e-6);
        EXPECT_NEAR(max.y(), rotated.maximum().y(), 1e-6);

        auto corners = rotated.corners();
        for(std::size_t i = 0 ; i < 4 ; ++i) {
            EXPECT_NEAR(expected_corners[i].x(), corners[i].x(), 1e-6);
            EXPECT_NEAR(expected_corners[i].y(), corners[i].y(), 1e-6);
        }

        Edges expected_edges;
        expected_edges[0][0] = expected_corners[0];
        expected_edges[0][1] = expected_corners[1];
        expected_edges[1][0] = expected_corners[1];
        expected_edges[1][1] = expected_corners[2];
        expected_edges[2][0] = expected_corners[2];
        expected_edges[2][1] = expected_corners[3];
        expected_edges[3][0] = expected_corners[3];
        expected_edges[3][1] = expected_corners[0];

        auto edges = rotated.edges();
        for(std::size_t i = 0 ; i < 4 ; ++i) {
            EXPECT_NEAR(expected_edges[i][0].x(), edges[i][0].x(), 1e-6);
            EXPECT_NEAR(expected_edges[i][0].y(), edges[i][0].y(), 1e-6);
            EXPECT_NEAR(expected_edges[i][1].x(), edges[i][1].x(), 1e-6);
            EXPECT_NEAR(expected_edges[i][1].y(), edges[i][1].y(), 1e-6);
        }

        rotated.transform(rot.inverse());

        EXPECT_NEAR(r.minimum().x(), rotated.minimum().x(), 1e-6);
        EXPECT_NEAR(r.minimum().y(), rotated.minimum().y(), 1e-6);
        EXPECT_NEAR(r.maximum().x(), rotated.maximum().x(), 1e-6);
        EXPECT_NEAR(r.maximum().y(), rotated.maximum().y(), 1e-6);
        expected_edges = r.edges();
        expected_corners = r.corners();

        corners = rotated.corners();
        for(std::size_t i = 0 ; i < 4 ; ++i) {
            EXPECT_NEAR(expected_corners[i].x(), corners[i].x(), 1e-6);
            EXPECT_NEAR(expected_corners[i].y(), corners[i].y(), 1e-6);
        }

        edges = rotated.edges();
        for(std::size_t i = 0 ; i < 4 ; ++i) {
            EXPECT_NEAR(expected_edges[i][0].x(), edges[i][0].x(), 1e-6);
            EXPECT_NEAR(expected_edges[i][0].y(), edges[i][0].y(), 1e-6);
            EXPECT_NEAR(expected_edges[i][1].x(), edges[i][1].x(), 1e-6);
            EXPECT_NEAR(expected_edges[i][1].y(), edges[i][1].y(), 1e-6);
        }
    }
    {
        Point min(2.0, -1.0);
        Point max = Point(3.0, 0.0);

        Points expected_corners =
        {
            min,
            Point(max.x(), min.y()),
            max,
            Point(min.x(), max.y())
        };

        BoundingRectangle moved = r;
        moved.transform(trans);

        EXPECT_NEAR(min.x(), moved.minimum().x(), 1e-6);
        EXPECT_NEAR(min.y(), moved.minimum().y(), 1e-6);
        EXPECT_NEAR(max.x(), moved.maximum().x(), 1e-6);
        EXPECT_NEAR(max.y(), moved.maximum().y(), 1e-6);

        auto corners = moved.corners();
        for(std::size_t i = 0 ; i < 4 ; ++i) {
            EXPECT_NEAR(expected_corners[i].x(), corners[i].x(), 1e-6);
            EXPECT_NEAR(expected_corners[i].y(), corners[i].y(), 1e-6);
        }

        Edges expected_edges;
        expected_edges[0][0] = expected_corners[0];
        expected_edges[0][1] = expected_corners[1];
        expected_edges[1][0] = expected_corners[1];
        expected_edges[1][1] = expected_corners[2];
        expected_edges[2][0] = expected_corners[2];
        expected_edges[2][1] = expected_corners[3];
        expected_edges[3][0] = expected_corners[3];
        expected_edges[3][1] = expected_corners[0];

        auto edges = moved.edges();
        for(std::size_t i = 0 ; i < 4 ; ++i) {
            EXPECT_NEAR(expected_edges[i][0].x(), edges[i][0].x(), 1e-6);
            EXPECT_NEAR(expected_edges[i][0].y(), edges[i][0].y(), 1e-6);
            EXPECT_NEAR(expected_edges[i][1].x(), edges[i][1].x(), 1e-6);
            EXPECT_NEAR(expected_edges[i][1].y(), edges[i][1].y(), 1e-6);
        }

        moved.transform(trans.inverse());

        EXPECT_NEAR(r.minimum().x(), moved.minimum().x(), 1e-6);
        EXPECT_NEAR(r.minimum().y(), moved.minimum().y(), 1e-6);
        EXPECT_NEAR(r.maximum().x(), moved.maximum().x(), 1e-6);
        EXPECT_NEAR(r.maximum().y(), moved.maximum().y(), 1e-6);
        expected_edges = r.edges();
        expected_corners = r.corners();

        corners = moved.corners();
        for(std::size_t i = 0 ; i < 4 ; ++i) {
            EXPECT_NEAR(expected_corners[i].x(), corners[i].x(), 1e-6);
            EXPECT_NEAR(expected_corners[i].y(), corners[i].y(), 1e-6);
        }

        edges = moved.edges();
        for(std::size_t i = 0 ; i < 4 ; ++i) {
            EXPECT_NEAR(expected_edges[i][0].x(), edges[i][0].x(), 1e-6);
            EXPECT_NEAR(expected_edges[i][0].y(), edges[i][0].y(), 1e-6);
            EXPECT_NEAR(expected_edges[i][1].x(), edges[i][1].x(), 1e-6);
            EXPECT_NEAR(expected_edges[i][1].y(), edges[i][1].y(), 1e-6);
        }
    }
    {
        Point min(2.0, -1.0);
        Point max = Point(2.0, std::sqrt(2.) - 1.0);

        Points expected_corners =
        {
            min,
            Point(0.5 * std::sqrt(2.)  + 2., 0.5 * std::sqrt(2.) - 1.),
            max,
            Point(-0.5 * std::sqrt(2.) + 2., 0.5 * std::sqrt(2.) - 1.)
        };

        BoundingRectangle transformed = r;
        transformed.transform(trans * rot);

        EXPECT_NEAR(min.x(), transformed.minimum().x(), 1e-6);
        EXPECT_NEAR(min.y(), transformed.minimum().y(), 1e-6);
        EXPECT_NEAR(max.x(), transformed.maximum().x(), 1e-6);
        EXPECT_NEAR(max.y(), transformed.maximum().y(), 1e-6);

        auto corners = transformed.corners();
        for(std::size_t i = 0 ; i < 4 ; ++i) {
            EXPECT_NEAR(expected_corners[i].x(), corners[i].x(), 1e-6);
            EXPECT_NEAR(expected_corners[i].y(), corners[i].y(), 1e-6);
        }

        Edges expected_edges;
        expected_edges[0][0] = expected_corners[0];
        expected_edges[0][1] = expected_corners[1];
        expected_edges[1][0] = expected_corners[1];
        expected_edges[1][1] = expected_corners[2];
        expected_edges[2][0] = expected_corners[2];
        expected_edges[2][1] = expected_corners[3];
        expected_edges[3][0] = expected_corners[3];
        expected_edges[3][1] = expected_corners[0];

        auto edges = transformed.edges();
        for(std::size_t i = 0 ; i < 4 ; ++i) {
            EXPECT_NEAR(expected_edges[i][0].x(), edges[i][0].x(), 1e-6);
            EXPECT_NEAR(expected_edges[i][0].y(), edges[i][0].y(), 1e-6);
            EXPECT_NEAR(expected_edges[i][1].x(), edges[i][1].x(), 1e-6);
            EXPECT_NEAR(expected_edges[i][1].y(), edges[i][1].y(), 1e-6);
        }

        transformed.transform((trans * rot).inverse());

        EXPECT_NEAR(r.minimum().x(), transformed.minimum().x(), 1e-6);
        EXPECT_NEAR(r.minimum().y(), transformed.minimum().y(), 1e-6);
        EXPECT_NEAR(r.maximum().x(), transformed.maximum().x(), 1e-6);
        EXPECT_NEAR(r.maximum().y(), transformed.maximum().y(), 1e-6);
        expected_edges = r.edges();
        expected_corners = r.corners();

        corners = transformed.corners();
        for(std::size_t i = 0 ; i < 4 ; ++i) {
            EXPECT_NEAR(expected_corners[i].x(), corners[i].x(), 1e-6);
            EXPECT_NEAR(expected_corners[i].y(), corners[i].y(), 1e-6);
        }

        edges = transformed.edges();
        for(std::size_t i = 0 ; i < 4 ; ++i) {
            EXPECT_NEAR(expected_edges[i][0].x(), edges[i][0].x(), 1e-6);
            EXPECT_NEAR(expected_edges[i][0].y(), edges[i][0].y(), 1e-6);
            EXPECT_NEAR(expected_edges[i][1].x(), edges[i][1].x(), 1e-6);
            EXPECT_NEAR(expected_edges[i][1].y(), edges[i][1].y(), 1e-6);
        }
    }
}

TEST(boundingRectangle, axisAlignedBounding)
{
    Point min;

    tf::Transform rot(tf::createQuaternionFromYaw(M_PI / 4.0));
    tf::Transform trans(tf::createIdentityQuaternion(),
                        tf::Vector3(2.0, -1.0, 0.0));

    BoundingRectangle r(min, tf::Vector3(1.0, 0.0, .0), tf::Vector3(0.0, 1.0, .0));
    {
        Point min = Point(-.5 * std::sqrt(2.), 0.0);
        Point max = Point(.5 * std::sqrt(2.),std::sqrt(2.));

        Points expected_corners =
        {
            min,
            Point(max.x(), min.y()),
            max,
            Point(min.x(), max.y())
        };

        Edges expected_edges;
        expected_edges[0][0] = expected_corners[0];
        expected_edges[0][1] = expected_corners[1];
        expected_edges[1][0] = expected_corners[1];
        expected_edges[1][1] = expected_corners[2];
        expected_edges[2][0] = expected_corners[2];
        expected_edges[2][1] = expected_corners[3];
        expected_edges[3][0] = expected_corners[3];
        expected_edges[3][1] = expected_corners[0];


        BoundingRectangle rotated = r;
        rotated.transform(rot);

        BoundingRectangle enclosing = rotated.axisAlignedEnclosingXY();

        auto corners = enclosing.corners();
        for(std::size_t i = 0 ; i < 4 ; ++i) {
            EXPECT_NEAR(expected_corners[i].x(), corners[i].x(), 1e-6);
            EXPECT_NEAR(expected_corners[i].y(), corners[i].y(), 1e-6);
        }
        auto edges = enclosing.edges();
        for(std::size_t i = 0 ; i < 4 ; ++i) {
            EXPECT_NEAR(expected_edges[i][0].x(), edges[i][0].x(), 1e-6);
            EXPECT_NEAR(expected_edges[i][0].y(), edges[i][0].y(), 1e-6);
            EXPECT_NEAR(expected_edges[i][1].x(), edges[i][1].x(), 1e-6);
            EXPECT_NEAR(expected_edges[i][1].y(), edges[i][1].y(), 1e-6);
        }

    }
    {
        Point min(2.0, -1.0);
        Point max = Point(3.0, 0.0);

        Points expected_corners =
        {
            min,
            Point(max.x(), min.y()),
            max,
            Point(min.x(), max.y())
        };

        Edges expected_edges;
        expected_edges[0][0] = expected_corners[0];
        expected_edges[0][1] = expected_corners[1];
        expected_edges[1][0] = expected_corners[1];
        expected_edges[1][1] = expected_corners[2];
        expected_edges[2][0] = expected_corners[2];
        expected_edges[2][1] = expected_corners[3];
        expected_edges[3][0] = expected_corners[3];
        expected_edges[3][1] = expected_corners[0];

        BoundingRectangle moved = r;
        moved.transform(trans);

        BoundingRectangle enclosing = moved.axisAlignedEnclosingXY();

        auto corners = enclosing.corners();
        for(std::size_t i = 0 ; i < 4 ; ++i) {
            EXPECT_NEAR(expected_corners[i].x(), corners[i].x(), 1e-6);
            EXPECT_NEAR(expected_corners[i].y(), corners[i].y(), 1e-6);
        }
        auto edges = enclosing.edges();
        for(std::size_t i = 0 ; i < 4 ; ++i) {
            EXPECT_NEAR(expected_edges[i][0].x(), edges[i][0].x(), 1e-6);
            EXPECT_NEAR(expected_edges[i][0].y(), edges[i][0].y(), 1e-6);
            EXPECT_NEAR(expected_edges[i][1].x(), edges[i][1].x(), 1e-6);
            EXPECT_NEAR(expected_edges[i][1].y(), edges[i][1].y(), 1e-6);
        }
    }
    {
        Point min = Point(-.5 * std::sqrt(2.) + 2.0, -1.0);
        Point max = Point( .5 * std::sqrt(2.) + 2.0, std::sqrt(2.) - 1.0);

        Points expected_corners =
        {
            min,
            Point(max.x(), min.y()),
            max,
            Point(min.x(), max.y())
        };

        Edges expected_edges;
        expected_edges[0][0] = expected_corners[0];
        expected_edges[0][1] = expected_corners[1];
        expected_edges[1][0] = expected_corners[1];
        expected_edges[1][1] = expected_corners[2];
        expected_edges[2][0] = expected_corners[2];
        expected_edges[2][1] = expected_corners[3];
        expected_edges[3][0] = expected_corners[3];
        expected_edges[3][1] = expected_corners[0];

        BoundingRectangle transformed = r;
        transformed.transform(trans * rot);

        BoundingRectangle enclosing = transformed.axisAlignedEnclosingXY();

        auto corners = enclosing.corners();
        for(std::size_t i = 0 ; i < 4 ; ++i) {
            EXPECT_NEAR(expected_corners[i].x(), corners[i].x(), 1e-6);
            EXPECT_NEAR(expected_corners[i].y(), corners[i].y(), 1e-6);
        }
        auto edges = enclosing.edges();
        for(std::size_t i = 0 ; i < 4 ; ++i) {
            EXPECT_NEAR(expected_edges[i][0].x(), edges[i][0].x(), 1e-6);
            EXPECT_NEAR(expected_edges[i][0].y(), edges[i][0].y(), 1e-6);
            EXPECT_NEAR(expected_edges[i][1].x(), edges[i][1].x(), 1e-6);
            EXPECT_NEAR(expected_edges[i][1].y(), edges[i][1].y(), 1e-6);
        }
    }
}



int main(int argc, char *argv[])
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
