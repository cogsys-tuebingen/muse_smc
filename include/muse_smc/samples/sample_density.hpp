#ifndef MUSE_SMC_SAMPLE_DENSITY_HPP
#define MUSE_SMC_SAMPLE_DENSITY_HPP

#include <memory>

namespace muse_smc {
template <typename sample_t>
class SampleDensity {
 public:
  using sample_density_t = SampleDensity<sample_t>;

  virtual ~SampleDensity() = default;
  virtual void clear() = 0;
  virtual void insert(const sample_t &sample) = 0;
  virtual void estimate() = 0;
};
}  // namespace muse_smc

#endif  // MUSE_SMC_SAMPLE_DENSITY_HPP
