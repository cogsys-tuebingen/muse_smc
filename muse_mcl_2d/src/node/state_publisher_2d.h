#ifndef STATE_PUBLISHER_H
#define STATE_PUBLISHER_H

#include <muse_smc/smc/smc_state.hpp>
#include <muse_smc/utility/csv_logger.hpp>

#include <muse_mcl_2d/samples/sample_2d.hpp>
#include <muse_mcl_2d/tf/tf_publisher.hpp>

#include "sample_set_publisher_2d.h"

namespace muse_mcl_2d {
class StatePublisher : public muse_smc::SMCState<Sample2D>
{
public:
    using Ptr = std::shared_ptr<StatePublisher>;

    StatePublisher();
    virtual ~StatePublisher();

    void setup(ros::NodeHandle &nh);

    virtual void publish(const typename sample_set_t::Ptr &sample_set) override;
    virtual void publishIntermidiate(const typename sample_set_t::Ptr &sample_set) override;

private:
    TFPublisher::Ptr            tf_publisher_;
    SampleSetPublisher2D::Ptr   sample_publisher_;

    std::string                 world_frame_;
    std::string                 odom_frame_;
    std::string                 base_frame_;

    math::StampedTransform2D    latest_w_T_b_;
    math::Covariance2D          latest_w_T_b_covariance_;

    void publishState(const typename sample_set_t::Ptr &sample_set);

};
}

#endif // STATE_PUBLISHER_H
