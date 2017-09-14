#include <muse_smc/time/time.hpp>
#include <muse_smc/math/random.hpp>
#include <muse_smc/math/angle.hpp>
#include <muse_smc/samples/sample_set.hpp>

#include <muse_mcl_2d/samples/sample_density_2d.hpp>
#include <muse_mcl_2d/samples/sample_2d.hpp>


#include <tf/tf.h>

void insertion(muse_smc::SampleSet<muse_mcl_2d::Sample2D> &set)
{
    muse_smc::math::random::Uniform<1> rng_l(-100.0, 100.0);
    muse_smc::math::random::Uniform<1> rng_a(-M_PI, M_PI);

    const std::size_t size = set.getMaximumSampleSize();
    std::vector<muse_mcl_2d::Sample2D> buff;
    for(std::size_t i = 0 ; i < size; ++i) {
        muse_mcl_2d::Sample2D s;
        s.state.tx() = rng_l.get();
        s.state.ty() = rng_l.get();
        s.state.setYaw(muse_smc::math::angle::normalize(rng_a.get()));
        buff.emplace_back(s);
    }



    muse_smc::Time start = muse_smc::Time::now();
    auto i = set.getInsertion();
    std::size_t  inserted = 0ul;
    while(i.canInsert()) {
        i.insert(buff[inserted]);
        ++inserted;
    }

    std::cout << "inserted : " << inserted << "\n";
    std::cout << "took time: " << (muse_smc::Time::now() - start).milliseconds() << "ms" << "\n";
}


void stateIteration(muse_smc::SampleSet<muse_mcl_2d::Sample2D> &set)
{
    auto interface = set.getStateIterator();
    muse_smc::Time start = muse_smc::Time::now();
    for(auto it = interface.begin() ; it != interface.end() ; ++it) {
        auto &state = (*it);
        state.tx() += 0.1;
        state.ty() += 0.1;
    }
    std::cout << "stateIteration took time:                 " << (muse_smc::Time::now() - start).milliseconds() << "ms" << "\n";
}

void stateIterationForEach(muse_smc::SampleSet<muse_mcl_2d::Sample2D> &set)
{
    auto interface = set.getStateIterator();
    muse_smc::Time start = muse_smc::Time::now();
    for(auto &state : interface) {
        state.tx() += 0.1;
        state.ty() += 0.1;
    }

    std::cout << "stateIterationForEach took time:          " << (muse_smc::Time::now() - start).milliseconds() << "ms" << "\n";
}

void weightIteration(muse_smc::SampleSet<muse_mcl_2d::Sample2D> &set)
{
    auto interface = set.getWeightIterator();
    muse_smc::Time start = muse_smc::Time::now();
    auto end = interface.end();
    for(auto it = interface.begin() ; it != end ; ++it) {
        auto &weight = (*it);
        weight = 0.01;
    }
    std::cout << "weightIteration took time:                " << (muse_smc::Time::now() - start).milliseconds() << "ms" << "\n";
}

void weightIterationForEach(muse_smc::SampleSet<muse_mcl_2d::Sample2D> &set)
{
    auto interface = set.getWeightIterator();
    muse_smc::Time start = muse_smc::Time::now();
    for(auto &weight : interface) {
        weight = 0.05;
    }
    std::cout << set.getMaximumWeight() << "\n";
    std::cout << "weightIterationForEach took time:         " << (muse_smc::Time::now() - start).milliseconds() << "ms" << "\n";
}

#include <muse_mcl_2d/odometry/odometry_2d.hpp>
int main(int argc, char *argv[])
{
    muse_mcl_2d::SampleIndexation2D  indexation({0.5, M_PI / 180.0 * 10.0});
    muse_mcl_2d::SampleDensity2D::Ptr density (new muse_mcl_2d::SampleDensity2D(indexation, 500000));
    muse_smc::SampleSet<muse_mcl_2d::Sample2D> set("frame",
                                                   muse_smc::Time::now(),
                                                   100,
                                                   500000,
                                                   density);


    std::cout << "Sizes of structs in use."         << "\n";
    std::cout << sizeof(tf::Transform)              << "\n";
    std::cout << sizeof(muse_mcl_2d::Sample2D)      << "\n";
    std::cout << sizeof(muse_mcl_2d::Transform2D)   << "\n";
    std::cout << sizeof(muse_mcl_2d::Odometry2D)    << "\n";

    insertion(set);
    stateIteration(set);
    stateIterationForEach(set);
    weightIteration(set);
    weightIterationForEach(set);

    return 0;
}
