#ifndef MIN_HEAP_HPP
#define MIN_HEAP_HPP

#include <etl/vector.h>
#include <flat_set>
#include <map>
#include <vector>

template <typename impl_type, typename distance_container_type, bool recurse = false> struct min_heap {
  using element_type = impl_type::element_type;
  using index_type = impl_type::index_type;
  using distance_type = impl_type::distance_type;
  template <typename... Args> inline min_heap(Args&&... args) : impl_(std::forward<Args>(args)...) {
    reset();
  }
  inline void reset() {
    distances_.reset();
    impl_.reset();
  }
  inline element_type top() {
    assert(impl_.size() >= 1ULL);
    return impl_.front(distance_for_index(0));
  }
  inline bool empty() const {
    return !(impl_.size() >= 1ULL);
  }
  inline void pop() {
    assert(impl_.size() >= 1);
    if (impl_.size() >= 2) [[likely]]
      impl_.swap(0, impl_.size() - 1);
    assert(impl_.size() >= 1);
    impl_.pop_back();
    shift_down(0);
  }
  inline bool push(const element_type& element) {
    distance_type new_distance = impl_.distance(element);
    if (distances_.if_distance_smaller_replace(impl_.name(element), new_distance)) [[unlikely]] {
      index_type index = impl_.find_index(element);
      if (index < impl_.size()) [[likely]] {
        shift_up(index, new_distance);
      } else {
        assert(index == impl_.max_index());
        impl_.push_back(element);
        shift_up(impl_.size() - 1, new_distance);
      }
      return true;
    }
    return false;
  }
  inline distance_type distance_for_index(index_type i) {
    return distances_.distance_for(impl_.name_at(i));
  }
  inline distance_type distance_for_element(const element_type& element) {
    return distances_.distance_for(impl_.name(element));
  }
  inline void shift_down(index_type current) {
    while (true) {
      index_type left = current * 2 + 1;
      index_type right = current * 2 + 2;
      if (left >= impl_.size()) [[unlikely]] {
        break;
      }
      distance_type left_dist = distance_for_index(left);
      distance_type right_dist = distances_.max_distance();
      if (right < impl_.size()) [[likely]] {
        right_dist = distance_for_index(right);
      }
      if (left_dist < right_dist) {
        if (left_dist < distance_for_index(current)) {
          impl_.swap(current, left);
          current = left;
        } else {
          break;
        }
      } else {
        if (right_dist < distance_for_index(current)) {
          impl_.swap(current, right);
          current = right;
        } else {
          break;
        }
      }
    }
  }
  inline void shift_up(index_type current, distance_type valueDist) {
    while (current > 0) {
      index_type parent = (current - 1) / 2;
      if (valueDist < distance_for_index(parent)) {
        impl_.swap(current, parent);
        current = parent;
      } else {
        break;
      }
    }
  }
  impl_type impl_;
  distance_container_type distances_;
};
template <size_t count> struct inverted_fixed_distance_container {
  inverted_fixed_distance_container() {}
  inline bool if_distance_smaller_replace(size_t k, uint64_t dist) {
    assert(k < distances.size());
    auto& ref = distances[k];
    if (dist < (~ref)) {
      ref = ~dist;
      return true;
    }
    return false;
  }
  inline uint64_t distance_for(size_t k) {
    assert(k < distances.size());
    return ~distances[k];
  }
  static constexpr uint64_t max_distance() {
    return -1ULL;
  }
  inline void reset() {
    distances.fill(0);
  }
  std::array<uint64_t, count> distances;
};
template <size_t count> struct fixed_distance_container {
  fixed_distance_container() {
    reset();
  }
  inline void reset() {
    distances.fill(-1ULL);
  }
  inline bool if_distance_smaller_replace(size_t k, uint64_t dist) {
    assert(k < distances.size());
    auto& ref = distances[k];
    if (dist < ref) {
      ref = dist;
      return true;
    }
    return false;
  }
  inline uint64_t distance_for(size_t k) {
    assert(k < distances.size());
    return distances[k];
  }
  static constexpr uint64_t max_distance() {
    return -1ULL;
  }
  std::array<uint64_t, count> distances;
};
template <size_t count> struct idx_heap_impl {
  using index_type = size_t;
  using distance_type = uint64_t;
  using element_type = std::pair<uint32_t, distance_type>;
  inline distance_type distance(element_type e) {
    return e.second;
  }
  inline uint32_t name(element_type e) {
    return e.first;
  }
  static constexpr uint32_t max_index() {
    return -1U;
  }
  etl::vector<uint32_t, count> elements;
  std::array<uint32_t, count> reverse;
  idx_heap_impl() {
    // elements.reserve(count);
  }
  inline void reset() {
    reverse.fill(max_index());
    elements.clear();
  }
  inline element_type front(distance_type d) {
    assert(elements.size());
    return {elements.front(), d};
  }
  inline void swap(index_type a, index_type b) {
    assert(a != b);
    assert(a < elements.size());
    assert(b < elements.size());
    assert(reverse.at(elements.at(a)) == a);
    assert(reverse.at(elements.at(b)) == b);
    assert(a < elements.size());
    assert(b < elements.size());
    std::swap(reverse[elements[a]], reverse[elements[b]]);
    std::swap(elements[a], elements[b]);
    assert(reverse.at(elements.at(a)) == a);
    assert(reverse.at(elements.at(b)) == b);
  }
  inline void pop_back() {
    assert(elements.size());
    assert(elements.back() < reverse.size());
    reverse[elements.back()] = max_index();
    elements.pop_back();
  }
  inline void push_back(element_type e) {
    assert(reverse.at(e.first) == max_index());
    assert(e.first < reverse.size());
    reverse[e.first] = elements.size();
    elements.push_back(e.first);
  }
  inline size_t name_at(index_type i) {
    assert(i < elements.size());
    return name({elements[i], max_index()});
  }
  inline index_type find_index(element_type e) {
    assert(e.first < reverse.size());
    return reverse[e.first];
  }
  inline index_type size() const {
    return elements.size();
  }
};

#endif // MIN_HEAP_HPP
