#include <muse_amcl/data_sources/tf_provider.hpp>

#include <gtest/gtest.h>
#include  <thread>

TEST(TestMuseAMCL, testTFProviderSingleThreaded)
{
    muse_amcl::TFProvider tf_provider;
    std::vector<std::string> frames;
    while(frames.size() != 3) {
        ros::Duration(0.5).sleep();
        tf_provider.getFrameStrings(frames);
    }

    const std::string world_frame = "world";
    const std::string frame0 = "map0";

    for(std::size_t i = 0 ; i < 100 ; ++i) {
        tf::Transform transform;
        bool success = tf_provider.lookupTransform(frame0,
                                                   world_frame,
                                                   ros::Time::now(),
                                                   transform,
                                                   ros::Duration(0.1));
        EXPECT_TRUE(success);
    }
}

TEST(TestMuseAMCL, testTFProviderMultiThreaded)
{
    muse_amcl::TFProvider tf_provider;
    std::vector<std::string> frames;
    while(frames.size() != 3) {
        ros::Duration(0.5).sleep();
        tf_provider.getFrameStrings(frames);
    }

    const std::string world_frame = "world";
    const std::string frame0 = "map0";
    const std::string frame1 = "map1";

    auto exec0 = [world_frame, frame0, &tf_provider] () {
        for(std::size_t i = 0 ; i < 100 ; ++i) {
            tf::Transform transform;
            EXPECT_TRUE(
                        tf_provider.lookupTransform(frame0,
                                                    world_frame,
                                                    ros::Time::now(),
                                                    transform,
                                                    ros::Duration(0.2)));
        }
    };
    auto exec1 = [world_frame, frame1, &tf_provider] () {
        for(std::size_t i = 0 ; i < 100 ; ++i) {
            tf::Transform transform;
            EXPECT_TRUE(
                        tf_provider.lookupTransform(frame1,
                                                    world_frame,
                                                    ros::Time::now(),
                                                    transform,
                                                    ros::Duration(0.2)));
        }
    };

    std::thread t0(exec0);
    std::thread t1(exec1);

    t0.join();
    t1.join();
}

TEST(TestMuseAMCL, testLockedTFProvider)
{
    muse_amcl::TFProvider tf_provider;
    std::vector<std::string> frames;
    while(frames.size() != 3) {
        ros::Duration(0.5).sleep();
        tf_provider.getFrameStrings(frames);
    }

    const std::string world_frame = "world";
    const std::string frame0 = "map0";
    const std::string frame1 = "map1";

    std::vector<int> execution_sequence;
    std::mutex execution_sequence_mutex;
    auto add_to_sequence = [&execution_sequence_mutex, &execution_sequence](const int i) {
        std::unique_lock<std::mutex> l(execution_sequence_mutex);
        execution_sequence.emplace_back(i);
    };

    auto exec0 = [world_frame, frame0, &tf_provider, add_to_sequence] () {
        muse_amcl::TFProvider::LockedTFProvider locked = tf_provider.getLockedTFProvider();
        for(std::size_t i = 0 ; i < 100 ; ++i) {
            tf::Transform transform;
            add_to_sequence(0);
            EXPECT_TRUE(
                        locked.lookupTransform(frame0,
                                               world_frame,
                                               ros::Time::now(),
                                               transform,
                                                ros::Duration(0.2)));
        }
    };
    auto exec1 = [world_frame, frame1, &tf_provider, add_to_sequence] () {
        for(std::size_t i = 0 ; i < 100 ; ++i) {
            tf::Transform transform;
            add_to_sequence(1);
            EXPECT_TRUE(tf_provider.lookupTransform(frame1,
                                                    world_frame,
                                                    ros::Time::now(),
                                                    transform,
                                                    ros::Duration(0.2)));
        }
    };

    std::thread t0(exec0);
    std::thread t1(exec1);

    t0.join();
    t1.join();

    EXPECT_EQ(200l, execution_sequence.size());
    std::size_t sequence_length = 0;
    bool expect_zero = false;

    for(auto t : execution_sequence) {
        expect_zero |= t==0;
        if(expect_zero && sequence_length < 100) {
            EXPECT_EQ(0, t);
            ++sequence_length;
        } else {
            EXPECT_EQ(1, t);
        }
    }

    std::cout << std::endl;
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "test_node_tf_provider");
    ros::Time::init();
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
