#include <muse_amcl/utils/signals.hpp>

#include <gtest/gtest.h>

TEST(signalsConnection, connectionCallback)
{
    using TestCallback = std::function<void(int,int)>;
    using TestSignal = muse_amcl::Signal<TestCallback>;

    auto callback = [](int a, int b){return;};
    TestSignal signal;
    TestSignal::Connection::Ptr connection = signal.connect(callback);
    EXPECT_FALSE(!connection);
}

TEST(signalsConnection, connectionCallbackNonVoid)
{
    using TestCallback = std::function<int(int,int)>;
    using TestSignal = muse_amcl::Signal<TestCallback>;

    auto callback = [](int a, int b){return 0;};
    TestSignal signal;
    TestSignal::Connection::Ptr connection = signal.connect(callback);
    EXPECT_FALSE(!connection);
}

TEST(signalsConnection, enabling)
{
    using TestCallback = std::function<void()>;
    using TestSignal = muse_amcl::Signal<TestCallback>;

    auto callback = [](){;};
    TestSignal signal;
    TestSignal::Connection::Ptr connection = signal.connect(callback);
    EXPECT_FALSE(!connection);
    EXPECT_FALSE(signal.isEnabled());
    signal.enable();
    EXPECT_TRUE(signal.isEnabled());
    signal.disable();
    EXPECT_FALSE(signal.isEnabled());
}

TEST(signalsConnection, callbackExecution)
{
    int result = 0;
    using TestCallback = std::function<void(int)>;
    using TestSignal = muse_amcl::Signal<TestCallback>;

    auto increment = [&result](int a){result += a;};
    TestSignal signal;
    TestSignal::Connection::Ptr connection = signal.connect(increment);
    EXPECT_FALSE(!connection);
    EXPECT_FALSE(signal.isEnabled());
    signal.enable();
    EXPECT_TRUE(signal.isEnabled());

    int expected = 0;
    for(std::size_t i = 0 ; i < 100 ; ++i) {
        signal(i);
        expected += i;
        EXPECT_EQ(expected, result);
    }
}

TEST(signalsConnection, enableDisable)
{
    int result = 0;
    using TestCallback = std::function<void(int)>;
    using TestSignal = muse_amcl::Signal<TestCallback>;

    auto increment = [&result](int a){result += a;};
    TestSignal signal;
    TestSignal::Connection::Ptr connection = signal.connect(increment);
    EXPECT_FALSE(!connection);
    EXPECT_FALSE(signal.isEnabled());
    signal.enable();
    EXPECT_TRUE(signal.isEnabled());

    bool enabled = false;
    int expected = 0;
    for(std::size_t i = 0 ; i < 100 ; ++i) {
        enabled = !enabled;

        if(enabled) {
            expected += i;
            signal.enable();
        } else {
            signal.disable();
        }
        signal(i);
        EXPECT_EQ(expected, result);
    }
}

TEST(signalsConnection, disconnectViaHandleReset)
{
    int result = 0;
    using TestCallback = std::function<void(int)>;
    using TestSignal = muse_amcl::Signal<TestCallback>;

    auto increment = [&result](int a){result += a;};
    TestSignal signal;
    TestSignal::Connection::Ptr connection = signal.connect(increment);
    EXPECT_FALSE(!connection);
    EXPECT_FALSE(signal.isEnabled());
    signal.enable();
    EXPECT_TRUE(signal.isEnabled());

    int expected = 0;
    for(std::size_t i = 0 ; i < 100 ; ++i) {
        expected += i;
        signal(i);
        EXPECT_EQ(expected, result);
    }
    connection.reset();
    for(std::size_t i = 1 ; i < 101 ; ++i) {
        expected += i;
        signal(i);
        EXPECT_GT(expected, result);
    }
}

TEST(signalsConnection, disconnectViaMethod)
{
    int result = 0;
    using TestCallback = std::function<void(int)>;
    using TestSignal = muse_amcl::Signal<TestCallback>;

    auto increment = [&result](int a){result += a;};
    TestSignal signal;
    TestSignal::Connection::Ptr connection = signal.connect(increment);
    EXPECT_FALSE(!connection);
    EXPECT_FALSE(signal.isEnabled());
    signal.enable();
    EXPECT_TRUE(signal.isEnabled());

    int expected = 0;
    for(std::size_t i = 0 ; i < 100 ; ++i) {
        expected += i;
        signal(i);
        EXPECT_EQ(expected, result);
    }
    signal.disconnect(connection);
    for(std::size_t i = 1 ; i < 101 ; ++i) {
        expected += i;
        signal(i);
        EXPECT_GT(expected, result);
    }
}

int main(int argc, char *argv[])
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
