#ifndef MUSE_SMC_SAMPLE_WEIGHT_ITERATOR_HPP
#define MUSE_SMC_SAMPLE_WEIGHT_ITERATOR_HPP

/// CSLIBS
#include <cslibs_utility/buffered/buffered_vector.hpp>
#include <cslibs_utility/common/delegate.hpp>

/// PROJECT
#include <muse_smc/smc/traits/sample.hpp>

namespace muse_smc {
template <typename Sample_T, typename State_T, typename Weight_T>
class WeightIterator
    : public std::iterator<std::random_access_iterator_tag, Weight_T> {
 public:
  using parent = std::iterator<std::random_access_iterator_tag, Weight_T>;
  using reference = typename parent::reference;
  using notify_update =
      cslibs_utility::common::delegate<void(const Weight_T &)>;

  inline explicit WeightIterator(Sample_T *begin, notify_update update)
      : data_(begin), update_(update) {}

  virtual ~WeightIterator() = default;

  inline WeightIterator &operator++() {
    update_(data_->weight());
    ++data_;
    return *this;
  }

  inline bool operator==(const WeightIterator &_other) const {
    return data_ == _other.data_;
  }

  inline bool operator!=(const WeightIterator &_other) const {
    return !(*this == _other);
  }

  inline reference operator*() const { return data_->weight(); }

  inline State_T const &state() const { return data_->state(); }

 private:
  Sample_T *data_{nullptr};
  notify_update update_;
};

template <typename Sample_T, typename State_T, typename Weight_T>
class WeightIteration {
 public:
  using sample_vector_t =
      cslibs_utility::buffered::buffered_vector<Sample_T,
                                                typename Sample_T::allocator_t>;
  using notify_update = cslibs_utility::common::delegate<void(const double)>;
  using notify_touch = cslibs_utility::common::delegate<void()>;
  using notify_finished = cslibs_utility::common::delegate<void()>;
  using iterator_t = WeightIterator<Sample_T, State_T, Weight_T>;
  using const_iterator_t = typename sample_vector_t::const_iterator;

  inline WeightIteration(sample_vector_t &data, notify_touch touch,
                         notify_update update, notify_finished finish)
      : data_(data),
        touch_(touch),
        update_(update),
        finish_(finish),
        untouched_(true) {}

  virtual ~WeightIteration() {
    if (!untouched_) finish_();
  }

  inline const_iterator_t const_begin() const { return data_.begin(); }

  inline const_iterator_t const_end() const { return data_.end(); }

  inline iterator_t begin() {
    if (untouched_) {
      untouched_ = false;
      touch_();
    }

    return iterator_t(&(*data_.begin()), update_);
  }

  inline iterator_t end() { return iterator_t(&(*data_.end()), update_); }

  inline std::size_t size() const { return data_.size(); }

  inline std::size_t capacity() const { return data_.capacity(); }

 private:
  sample_vector_t &data_;
  notify_touch touch_;
  notify_update update_;
  notify_finished finish_;
  bool untouched_;
};
}  // namespace muse_smc

#endif  // MUSE_SMC_SAMPLE_WEIGHT_ITERATOR_HPP
