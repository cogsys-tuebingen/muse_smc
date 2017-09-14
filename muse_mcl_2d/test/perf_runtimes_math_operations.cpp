#include <tf/tf.h>

#include <muse_smc/time/time.hpp>
#include <muse_smc/math/random.hpp>
#include <iomanip>

#include <muse_mcl_2d/math/transform_2d.hpp>
#include <muse_mcl_2d/math/vector_2d.hpp>
#include <muse_smc/utility/stamped.hpp>
#include <muse_smc/math/angle.hpp>

namespace muse_mcl_2d {
class Transform2DLegacy {
public:
    inline Transform2DLegacy() :
        translation_(0.0, 0.0),
        yaw_(0.0),
        sin_(0.0),
        cos_(1.0)
    {
    }

    inline Transform2DLegacy(const double x,
                       const double y) :
        translation_(x, y),
        yaw_(0.0),
        sin_(0.0),
        cos_(1.0)
    {
    }

    inline Transform2DLegacy(const Vector2D &translation) :
        translation_(translation),
        yaw_(0.0),
        sin_(0.0),
        cos_(1.0)
    {
    }

    inline Transform2DLegacy(const double x,
                       const double y,
                       const double yaw) :
        translation_(x, y),
        yaw_(yaw),
        sin_(std::sin(yaw_)),
        cos_(std::cos(yaw_))
    {
    }

    inline Transform2DLegacy(const Vector2D &translation,
                       const double yaw) :
        translation_(translation),
        yaw_(yaw),
        sin_(std::sin(yaw_)),
        cos_(std::cos(yaw_))
    {
    }

    inline Transform2DLegacy(const Transform2DLegacy &other) :
        translation_(other.translation_),
        yaw_(other.yaw_),
        sin_(other.sin_),
        cos_(other.cos_)
    {
    }

    inline Transform2DLegacy(Transform2DLegacy &&other) :
        translation_(other.translation_),
        yaw_(other.yaw_),
        sin_(other.sin_),
        cos_(other.cos_)
    {
    }

    inline Vector2D operator * (const Vector2D &v) const
    {
        return Vector2D(cos_ * v.x() - sin_ * v.y() + translation_.x(),
                        sin_ * v.x() + cos_ * v.y() + translation_.y());

    }

    inline Transform2DLegacy operator * (const Transform2DLegacy &other) const
    {
        Transform2DLegacy t;
        t.setYaw(muse_smc::math::angle::normalize(yaw_ + other.yaw_));
        t.translation_.x() = cos_ * other.translation_.x() - sin_ * other.translation_.y() + translation_.x();
        t.translation_.y() = sin_ * other.translation_.x() + cos_ * other.translation_.y() + translation_.y();
        return t;
    }


    inline Transform2DLegacy & operator *= (const Transform2DLegacy &other)
    {
        translation_.x() = cos_ * other.translation_.x() - sin_ * other.translation_.y() + translation_.x();
        translation_.y() = sin_ * other.translation_.x() + cos_ * other.translation_.y() + translation_.y();
        setYaw(muse_smc::math::angle::normalize(yaw_ + other.yaw_));
        return *this;
    }

    inline Transform2DLegacy& operator = (const Transform2DLegacy &other)
    {
        if(&other != this) {
            yaw_ = other.yaw_;
            sin_ = other.sin_;
            cos_ = other.cos_;
            translation_ = other.translation_;
        }
        return *this;
    }

    inline Transform2DLegacy& operator = (Transform2DLegacy &&other)
    {
        if(&other != this) {
            yaw_ = other.yaw_;
            sin_ = other.sin_;
            cos_ = other.cos_;
            translation_ = other.translation_;
        }
        return *this;
    }

    inline Transform2DLegacy inverse() const
    {
        Transform2DLegacy t;
        t.setYaw(-yaw_);
        t.translation_ = -(t * translation_);
        return t;
    }

    inline Transform2DLegacy operator -() const
    {
        return inverse();
    }

    inline double & tx()
    {
        return translation_.x();
    }

    inline double tx() const
    {
        return translation_.x();
    }

    inline double & ty()
    {
        return translation_.y();
    }

    inline double ty() const
    {
        return translation_.y();
    }

    inline Vector2D & translation()
    {
        return translation_;
    }

    inline Vector2D const & translation() const
    {
        return translation_;
    }

    inline void setYaw(const double yaw)
    {
        yaw_ = yaw;
        sin_ = std::sin(yaw_);
        cos_ = std::cos(yaw_);
    }

    inline double yaw() const
    {
        return yaw_;
    }

    inline double sin() const
    {
        return sin_;
    }

    inline double cos() const
    {
        return cos_;
    }

    inline Eigen::Vector3d toEigen() const
    {
        return Eigen::Vector3d(translation_.x(), translation_.y(), yaw_);
    }

    inline void setFrom(const Eigen::Vector3d &eigen)
    {
        translation_.x() = eigen(0);
        translation_.y() = eigen(1);
        yaw_ = eigen(2);
        sin_ = std::sin(yaw_);
        cos_ = std::cos(yaw_);
    }

    inline void setFrom(const double x,
                        const double y,
                        const double yaw)
    {
        translation_.x() = x;
        translation_.y() = y;
        yaw_ = yaw;
        sin_ = std::sin(yaw_);
        cos_ = std::cos(yaw_);
    }

    inline Transform2DLegacy interpolate(const Transform2DLegacy &other,
                                   const double ratio) const
    {
        assert(ratio  >= 0.0);
        assert(ratio <= 1.0);
        if(ratio == 0.0) {
            return *this;
        }
        if(ratio == 1.0) {
            return other;
        }

        const  double ratio_inverse = 1.0 - ratio;
        const  Vector2D translation = translation_ * ratio_inverse + other.translation_ * ratio;
        const  double   yaw = muse_smc::math::angle::normalize(yaw_ * ratio_inverse + other.yaw_ * ratio);
        return Transform2DLegacy(translation, yaw);
    }

private:
    Vector2D translation_;
    double   yaw_;
    double   sin_;
    double   cos_;
};

inline std::ostream & operator << (std::ostream &out, const muse_mcl_2d::Transform2DLegacy &t)
{
    out << "[" << t.tx() << "," << t.ty() << "," << t.yaw() << "]";
    return out;
}

using StampedTransform2DLegacy = muse_smc::Stamped<Transform2DLegacy>;
}


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