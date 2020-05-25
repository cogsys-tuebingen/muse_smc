#ifndef MUSE_SMC_STATE_PUBLISHER_HPP
#define MUSE_SMC_STATE_PUBLISHER_HPP

namespace muse_smc {
/**
 * @brief The SMCState class is used to communicate the filter state to the
 * outside world. E.g. for the ROS use case one would publish the sample set and
 * the mean.
 * @brief state_space_description_t     - the state space description applying
 * to a given problem.
 */
template <typename sample_set_t>
class StatePublisher {
 public:
  using Ptr = std::shared_ptr<StatePublisher>;

  /**
   * @brief Publish a valid belief state, after resampling.
   * @param sample_set
   */
  virtual void publish(const typename sample_set_t::ConstPtr &sample_set) = 0;
  /**
   * @brief Publish an intermediate state of the filter, when resampling was not
   * applied for getting a valid belief state.
   * @param sample_set
   */
  virtual void publishIntermediate(
      const typename sample_set_t::ConstPtr &sample_set) = 0;
  /**
   * @brief publishConstant
   * @param sample_set
   */
  virtual void publishConstant(
      const typename sample_set_t::ConstPtr &sample_set) = 0;
};
}  // namespace muse_smc

#endif  // MUSE_SMC_STATE_PUBLISHER_HPP
