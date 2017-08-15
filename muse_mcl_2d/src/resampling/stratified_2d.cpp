#include <muse_smc/resampling/impl/stratified.hpp>

#include <muse_mcl_2d/resampling/resampling_2d.hpp>

namespace muse_mcl_2d {
class Stratified : public Resampling2D
{
public:

protected:
    virtual void doSetup(ros::NodeHandle &nh) override
    {
    }

    virtual void doApply(sample_set_t &sample_set) override
    {
        muse_smc::impl::Stratified<Sample2D>::apply(sample_set);
    }

    virtual void doApplyRecovery(sample_set_t &sample_set) override
    {
        muse_smc::impl::Stratified<Sample2D>::applyRecovery(uniform_pose_sampler_,
                                                            recovery_random_pose_probability_,
                                                            sample_set);
    }
};
}

CLASS_LOADER_REGISTER_CLASS(muse_mcl_2d::Stratified, muse_mcl_2d::Resampling2D)
