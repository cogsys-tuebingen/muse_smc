#include <muse_mcl_2d/math/transform_2d.hpp>
#include "transform_2d_legacy.hpp"
#include <tf/tf.h>

#include <muse_smc/time/time.hpp>
#include <muse_smc/math/random.hpp>
#include <iomanip>

const std::size_t ITERATIONS = 1000000;

void constructors()
{
    muse_smc::math::random::Uniform<1> rng(-10.0, 10.0);

    muse_smc::Time start = muse_smc::Time::now();
    double yaw = 0.0;
    for(std::size_t i = 0 ; i < ITERATIONS ; ++i) {
        muse_mcl_2d::Transform2D t;
        yaw = t.yaw();
    }
    std::cout << "empty:" << "\n";
    std::cout << "took time: " << (muse_smc::Time::now() - start).milliseconds() << "ms" << "\n";

    start = muse_smc::Time::now();
    for(std::size_t i = 0 ; i < ITERATIONS ; ++i) {
        muse_mcl_2d::Transform2D t(i, i);
        yaw = t.yaw();
    }
    std::cout << "x y:" << "\n";
    std::cout << "took time: " << (muse_smc::Time::now() - start).milliseconds() << "ms" << "\n";

    start = muse_smc::Time::now();
    muse_mcl_2d::Vector2D v(rng.get(), rng.get());
    for(std::size_t i = 0 ; i < ITERATIONS ; ++i) {
        muse_mcl_2d::Transform2D t(v);
        yaw = t.yaw();
        v.x() += i;
    }
    std::cout << "v:" << "\n";
    std::cout << "took time: " << (muse_smc::Time::now() - start).milliseconds() << "ms" << "\n";

    start = muse_smc::Time::now();
    for(std::size_t i = 0 ; i < ITERATIONS ; ++i) {
        muse_mcl_2d::Transform2D t(i,i,i);
        yaw = t.yaw();
    }
    std::cout << "x y yaw:" << "\n";
    std::cout << "took time: " << (muse_smc::Time::now() - start).milliseconds() << "ms" << "\n";

    start = muse_smc::Time::now();
    for(std::size_t i = 0 ; i < ITERATIONS ; ++i) {
        muse_mcl_2d::Transform2D t(v,i);
        yaw = t.yaw();
    }
    std::cout << "v yaw:" << "\n";
    std::cout << "took time: " << (muse_smc::Time::now() - start).milliseconds() << "ms" << "\n";

    start = muse_smc::Time::now();
    muse_mcl_2d::Transform2D t(rng.get(), rng.get());
    for(std::size_t i = 0 ; i < ITERATIONS ; ++i) {
        muse_mcl_2d::Transform2D t_(t);
        yaw = t_.yaw();
    }
    std::cout << "t:" << "\n";
    std::cout << "took time: " << (muse_smc::Time::now() - start).milliseconds() << "ms" << "\n";
}

void multiplyVector()
{
    muse_smc::math::random::Uniform<1> rng(-10.0, 10.0);
    double mean_ms_t = 0.0;
    double mean_ms_tl= 0.0;
    double mean_ms_tf= 0.0;

    for(std::size_t i = 0 ; i < ITERATIONS ; ++i) {
        muse_smc::Time start = muse_smc::Time::now();
        muse_mcl_2d::Transform2D t(rng.get(), rng.get(), muse_smc::math::angle::normalize(rng.get()));
        muse_mcl_2d::Vector2D v(rng.get(), rng.get());
        for(std::size_t i = 0 ; i < ITERATIONS ; ++i) {
            v = t * v;
        }
        mean_ms_t += (muse_smc::Time::now() - start).milliseconds();

        start = muse_smc::Time::now();
        muse_mcl_2d::Transform2DLegacy tl(rng.get(), rng.get(), muse_smc::math::angle::normalize(rng.get()));
        muse_mcl_2d::Vector2D tv(rng.get(), rng.get());
        for(std::size_t i = 0 ; i < ITERATIONS ; ++i) {
            tv = tl * tv;
        }
        mean_ms_tl += (muse_smc::Time::now() - start).milliseconds();

        start = muse_smc::Time::now();
        tf::Transform tf_t(tf::createQuaternionFromYaw(muse_smc::math::angle::normalize(rng.get())),
                           tf::Vector3(rng.get(), rng.get(), 0.0));
        tf::Vector3   tf_v (rng.get(), rng.get(), 0.0);
        for(std::size_t i = 0 ; i < ITERATIONS ; ++i) {
            tf_v = tf_t * tf_v;
        }
        mean_ms_tf += (muse_smc::Time::now() - start).milliseconds();
    }

    std::cout << "vector:" << "\n";
    std::cout << "took time: " << std::fixed << std::setprecision(10) << mean_ms_t / ITERATIONS << "ms" << "\n";
    std::cout << "legacy vector:" << "\n";
    std::cout << "took time: " << std::fixed << std::setprecision(10) << mean_ms_tl/ ITERATIONS << "ms" << "\n";
    std::cout << "tf vector:"  << "\n";
    std::cout << "took time: " << std::fixed << std::setprecision(10) << mean_ms_tf/ ITERATIONS << "ms" << "\n";
}

void multiplyTransform()
{
    muse_smc::math::random::Uniform<1> rng(-10.0, 10.0);
    double mean_ms_t = 0.0;
    double mean_ms_tl= 0.0;
    double mean_ms_tf= 0.0;

    muse_mcl_2d::Transform2D t;
    muse_mcl_2d::Transform2DLegacy tl;
    tf::Transform tf;
    for(std::size_t i = 0 ; i < ITERATIONS ; ++i) {
        muse_smc::Time start = muse_smc::Time::now();
        muse_mcl_2d::Transform2D ta(rng.get(), rng.get(), muse_smc::math::angle::normalize(rng.get()));
        muse_mcl_2d::Transform2D tb(rng.get(), rng.get(), muse_smc::math::angle::normalize(rng.get()));
        for(std::size_t i = 0 ; i < ITERATIONS ; ++i) {
            tb = ta * tb;
        }
        mean_ms_t += (muse_smc::Time::now() - start).milliseconds();
        t = tb;

        start = muse_smc::Time::now();
        muse_mcl_2d::Transform2DLegacy tla(rng.get(), rng.get(), muse_smc::math::angle::normalize(rng.get()));
        muse_mcl_2d::Transform2DLegacy tlb(rng.get(), rng.get(), muse_smc::math::angle::normalize(rng.get()));
        for(std::size_t i = 0 ; i < ITERATIONS ; ++i) {
            tlb = tla * tlb;
        }
        mean_ms_tl += (muse_smc::Time::now() - start).milliseconds();
        tl = tlb;

        start = muse_smc::Time::now();
        tf::Transform tf_ta(tf::createQuaternionFromYaw(muse_smc::math::angle::normalize(rng.get())),
                            tf::Vector3(rng.get(), rng.get(), 0.0));
        tf::Transform tf_tb(tf::createQuaternionFromYaw(muse_smc::math::angle::normalize(rng.get())),
                            tf::Vector3(rng.get(), rng.get(), 0.0));
        for(std::size_t i = 0 ; i < ITERATIONS ; ++i) {
            tf_tb = tf_ta * tf_tb;
        }
        mean_ms_tf += (muse_smc::Time::now() - start).milliseconds();
        tf = tf_tb;
    }

    std::cout << "transform multiply:" << "\n";
    std::cout << "took time: " << std::fixed << std::setprecision(10) << mean_ms_t / ITERATIONS << "ms" << "\n";
    std::cout << "legacy transform multiply:" << "\n";
    std::cout << "took time: " << std::fixed << std::setprecision(10) << mean_ms_tl/ ITERATIONS << "ms" << "\n";
    std::cout << "tf transform multiply:"  << "\n";
    std::cout << "took time: " << std::fixed << std::setprecision(10) << mean_ms_tf/ ITERATIONS << "ms" << "\n";
}

void multiplyAssignTransform()
{
    muse_smc::math::random::Uniform<1> rng(-10.0, 10.0);
    double mean_ms_t = 0.0;
    double mean_ms_tl= 0.0;
    double mean_ms_tf= 0.0;

    muse_mcl_2d::Transform2D t;
    muse_mcl_2d::Transform2DLegacy tl;
    tf::Transform tf;
    for(std::size_t i = 0 ; i < ITERATIONS ; ++i) {
        muse_smc::Time start = muse_smc::Time::now();
        muse_mcl_2d::Transform2D ta(rng.get(), rng.get(), muse_smc::math::angle::normalize(rng.get()));
        muse_mcl_2d::Transform2D tb(rng.get(), rng.get(), muse_smc::math::angle::normalize(rng.get()));
        for(std::size_t i = 0 ; i < ITERATIONS ; ++i) {
            tb *= ta;
        }
        mean_ms_t += (muse_smc::Time::now() - start).milliseconds();
        t = tb;

        start = muse_smc::Time::now();
        muse_mcl_2d::Transform2DLegacy tla(rng.get(), rng.get(), muse_smc::math::angle::normalize(rng.get()));
        muse_mcl_2d::Transform2DLegacy tlb(rng.get(), rng.get(), muse_smc::math::angle::normalize(rng.get()));
        for(std::size_t i = 0 ; i < ITERATIONS ; ++i) {
            tlb *= tla;
        }
        mean_ms_tl += (muse_smc::Time::now() - start).milliseconds();
        tl = tlb;

        start = muse_smc::Time::now();
        tf::Transform tf_ta(tf::createQuaternionFromYaw(muse_smc::math::angle::normalize(rng.get())),
                            tf::Vector3(rng.get(), rng.get(), 0.0));
        tf::Transform tf_tb(tf::createQuaternionFromYaw(muse_smc::math::angle::normalize(rng.get())),
                            tf::Vector3(rng.get(), rng.get(), 0.0));
        for(std::size_t i = 0 ; i < ITERATIONS ; ++i) {
            tf_tb *= tf_ta;
        }
        mean_ms_tf += (muse_smc::Time::now() - start).milliseconds();
        tf = tf_tb;
    }

    std::cout << "transform multiply assign:" << "\n";
    std::cout << "took time: " << std::fixed << std::setprecision(10) << mean_ms_t / ITERATIONS << "ms" << "\n";
    std::cout << "legacy transform multiply assign:" << "\n";
    std::cout << "took time: " << std::fixed << std::setprecision(10) << mean_ms_tl/ ITERATIONS << "ms" << "\n";
    std::cout << "tf transform multiply assign:"  << "\n";
    std::cout << "took time: " << std::fixed << std::setprecision(10) << mean_ms_tf/ ITERATIONS << "ms" << "\n";
}

void assign()
{
    muse_smc::math::random::Uniform<1> rng(-10.0, 10.0);
    double mean_ms_t = 0.0;
    double mean_ms_tl= 0.0;
    double mean_ms_tf= 0.0;

    muse_mcl_2d::Transform2D t;
    muse_mcl_2d::Transform2DLegacy tl;
    tf::Transform tf;
    for(std::size_t i = 0 ; i < ITERATIONS ; ++i) {
        muse_smc::Time start = muse_smc::Time::now();
        muse_mcl_2d::Transform2D ta(rng.get(), rng.get(), muse_smc::math::angle::normalize(rng.get()));
        muse_mcl_2d::Transform2D tb(rng.get(), rng.get(), muse_smc::math::angle::normalize(rng.get()));
        for(std::size_t i = 0 ; i < ITERATIONS ; ++i) {
            tb = ta;
        }
        mean_ms_t += (muse_smc::Time::now() - start).milliseconds();
        t = tb;

        start = muse_smc::Time::now();
        muse_mcl_2d::Transform2DLegacy tla(rng.get(), rng.get(), muse_smc::math::angle::normalize(rng.get()));
        muse_mcl_2d::Transform2DLegacy tlb(rng.get(), rng.get(), muse_smc::math::angle::normalize(rng.get()));
        for(std::size_t i = 0 ; i < ITERATIONS ; ++i) {
            tlb = tla;
        }
        mean_ms_tl += (muse_smc::Time::now() - start).milliseconds();
        tl = tlb;

        start = muse_smc::Time::now();
        tf::Transform tf_ta(tf::createQuaternionFromYaw(muse_smc::math::angle::normalize(rng.get())),
                            tf::Vector3(rng.get(), rng.get(), 0.0));
        tf::Transform tf_tb(tf::createQuaternionFromYaw(muse_smc::math::angle::normalize(rng.get())),
                            tf::Vector3(rng.get(), rng.get(), 0.0));
        for(std::size_t i = 0 ; i < ITERATIONS ; ++i) {
            tf_tb = tf_ta;
        }
        mean_ms_tf += (muse_smc::Time::now() - start).milliseconds();
        tf = tf_tb;
    }

    std::cout << "transform assign:" << "\n";
    std::cout << "took time: " << std::fixed << std::setprecision(10) << mean_ms_t / ITERATIONS << "ms" << "\n";
    std::cout << "legacy transform assign:" << "\n";
    std::cout << "took time: " << std::fixed << std::setprecision(10) << mean_ms_tl/ ITERATIONS << "ms" << "\n";
    std::cout << "tf transform assign:"  << "\n";
    std::cout << "took time: " << std::fixed << std::setprecision(10) << mean_ms_tf/ ITERATIONS << "ms" << "\n";
}

void inverse()
{
    muse_smc::math::random::Uniform<1> rng(-10.0, 10.0);
    double mean_ms_t = 0.0;
    double mean_ms_tl= 0.0;
    double mean_ms_tf= 0.0;

    muse_mcl_2d::Transform2D t;
    muse_mcl_2d::Transform2DLegacy tl;
    tf::Transform tf;
    for(std::size_t i = 0 ; i < ITERATIONS ; ++i) {
        muse_smc::Time start = muse_smc::Time::now();
        muse_mcl_2d::Transform2D ta(rng.get(), rng.get(), muse_smc::math::angle::normalize(rng.get()));
        for(std::size_t i = 0 ; i < ITERATIONS ; ++i) {
            t = ta.inverse() * t;
        }
        mean_ms_t += (muse_smc::Time::now() - start).milliseconds();

        start = muse_smc::Time::now();
        muse_mcl_2d::Transform2DLegacy tla(rng.get(), rng.get(), muse_smc::math::angle::normalize(rng.get()));
        for(std::size_t i = 0 ; i < ITERATIONS ; ++i) {
            tl = tla.inverse() * tl;
        }
        mean_ms_tl += (muse_smc::Time::now() - start).milliseconds();

        start = muse_smc::Time::now();
        tf::Transform tf_ta(tf::createQuaternionFromYaw(muse_smc::math::angle::normalize(rng.get())),
                            tf::Vector3(rng.get(), rng.get(), 0.0));
        for(std::size_t i = 0 ; i < ITERATIONS ; ++i) {
            tf = tf_ta.inverse() * tf;
        }
        mean_ms_tf += (muse_smc::Time::now() - start).milliseconds();
    }

    std::cout << "transform inverse:" << "\n";
    std::cout << "took time: " << std::fixed << std::setprecision(10) << mean_ms_t / ITERATIONS << "ms" << "\n";
    std::cout << "legacy transform inverse:" << "\n";
    std::cout << "took time: " << std::fixed << std::setprecision(10) << mean_ms_tl/ ITERATIONS << "ms" << "\n";
    std::cout << "tf transform inverse:"  << "\n";
    std::cout << "took time: " << std::fixed << std::setprecision(10) << mean_ms_tf/ ITERATIONS << "ms" << "\n";
}



int main(int argc, char *argv[])
{
    constructors();
    multiplyVector();
    multiplyTransform();
    multiplyAssignTransform();
    assign();
    inverse();
    return 0;
}
