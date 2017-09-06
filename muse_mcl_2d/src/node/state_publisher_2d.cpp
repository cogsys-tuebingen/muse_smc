#include "state_publisher_2d.h"

#include <muse_mcl_2d/samples/sample_density_2d.hpp>

using namespace muse_mcl_2d;

StatePublisher::StatePublisher() :
    latest_w_T_b_(Transform2D(),
                  muse_smc::Time(ros::Time::now().toNSec()))
{
}

StatePublisher::~StatePublisher()
{
}

void StatePublisher::setup(ros::NodeHandle &nh)
{
    const double pub_rate_state = nh.param<double>("pub_rate_state", 10.0);
    const double pub_rate_tf    = nh.param<double>("pub_rate_tf", 30.0);

    world_frame_ = nh.param<std::string>("world_frame", "/world");
    odom_frame_  = nh.param<std::string>("odom_frame", "/odom");
    base_frame_  = nh.param<std::string>("base_frame", "/base_link");

    sample_publisher_.reset(new SampleSetPublisher2D);
    sample_publisher_->setup(nh);
    sample_publisher_->start();

    tf_publisher_.reset(new TFPublisher(pub_rate_tf, odom_frame_, base_frame_, world_frame_/*, tf_timeout */));
    tf_publisher_->start();

    if(pub_rate_state != 0.0) {
        cycle_time_state_publication_ = ros::Duration(1.0 / pub_rate_state);
    }

    last_state_publication_ = ros::Time::now();
}

void StatePublisher::publish(const sample_set_t::Ptr &sample_set)
{
    /// calculate the latest transformation
    SampleDensity2D::ConstPtr density =
            std::dynamic_pointer_cast<SampleDensity2D const>(sample_set->getDensity());

    if(!density) {
        std::cerr << "[StatePublisher]: Incomaptible sample density estimation!" << "\n";
        return;
    }

    const SampleDensity2D::cluster_map_t      &clusters      = density->clusters();
    const SampleDensity2D::distribution_map_t &distributions = density->clusterDistributions();
    const SampleDensity2D::angular_mean_map_t &angular_means = density->clusterAngularMeans();

    double max_weight = std::numeric_limits<double>::lowest();
    int    max_cluster_id = -1;
    for(const auto &cluster : clusters) {
        const int cluster_id = cluster.first;
        const auto &distribution = distributions.at(cluster_id);
        const auto weight = distribution.getWeight();
        if(weight > max_weight) {
            max_cluster_id = cluster_id;
            max_weight = weight;
        }
    }
    if(max_cluster_id != -1) {
        const Eigen::Vector2d  position_mean = distributions.at(max_cluster_id).getMean();
        const Eigen::Matrix2d  position_covariance = distributions.at(max_cluster_id).getCovariance();
        const double           angular_mean  = angular_means.at(max_cluster_id).getMean();
        const double           angular_covariance = angular_means.at(max_cluster_id).getCovariance();
        latest_w_T_b_.data().tx() = position_mean(0);
        latest_w_T_b_.data().ty() = position_mean(1);
        latest_w_T_b_.data().setYaw(angular_mean);
        latest_w_T_b_.stamp() = sample_set->getStamp();
        latest_w_T_b_covariance_(0,0) = position_covariance(0,0);
        latest_w_T_b_covariance_(0,1) = position_covariance(0,1);
        latest_w_T_b_covariance_(1,0) = position_covariance(1,0);
        latest_w_T_b_covariance_(1,1) = position_covariance(1,1);
        latest_w_T_b_covariance_(2,2) = angular_covariance;

        /// make sure that TF gets published first #most important
        tf_publisher_->setTransform(latest_w_T_b_);
    }
    /// publish the particle set state
    publishState(sample_set);
}

void StatePublisher::publishIntermidiate(const sample_set_t::Ptr &sample_set)
{
    const ros::Time now = ros::Time::now();
    const ros::Time expected = last_state_publication_ + cycle_time_state_publication_;
    if(now > expected) {
        last_state_publication_ = expected;
        publishState(sample_set);
    }
}

void StatePublisher::publishState(const sample_set_t::Ptr &sample_set)
{
    sample_publisher_->add(sample_set->getSamples(),
                           sample_set->getMaximumWeight(),
                           latest_w_T_b_,
                           latest_w_T_b_covariance_,
                           sample_set->getStamp());
}
