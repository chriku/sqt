#ifndef MIN_HEAP_HPP
#define MIN_HEAP_HPP

#include "sqt.hpp"
#include <map>
#include <vector>

template <typename impl_type, typename distance_container_type> struct min_heap {
  using element_type = impl_type::element_type;
  using index_type = impl_type::index_type;
  using distance_type = impl_type::distance_type;
  inline auto top() {
    return impl_.front(distance_for_index(0));
  }
  inline bool empty() const {
    return !impl_.size();
  }
  inline void pop() {
    impl_.swap(0, impl_.size() - 1);
    impl_.pop_back();
    shift_down(0);
  }
  inline bool push(const element_type& element) {
    distance_type new_distance = impl_.distance(element);
    if (distances_.if_distance_smaller_replace(impl_.name(element), new_distance)) {
      index_type index = impl_.find_index(element);
      if (index < impl_.size()) {
        shift_up(index, new_distance);
      } else {
        assert(index == -1);
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
  inline void shift_down(index_type current) {
    while (true) {
      index_type left = current * 2 + 1;
      index_type right = current * 2 + 2;
      if (left >= impl_.size()) {
        break;
      }
      distance_type left_dist = distance_for_index(left);
      distance_type right_dist = -1ULL;
      if (right < impl_.size()) {
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
template <typename Key> struct map_distance_container {
  inline bool if_distance_smaller_replace(const Key& k, uint64_t dist) {
    auto [it, inserted] = distances.emplace(k, dist);
    if (inserted)
      return true;
    else if (dist < it->second) {
      it->second = dist;
      return true;
    } else
      return false;
  }
  inline uint64_t distance_for(const Key& k) {
    return distances.at(k);
  }
  inline uint64_t distance_for2(const Key& k) {
    auto it = distances.find(k);
    if (it != distances.end())
      return it->second;
    else
      return -1ULL;
  }
  std::map<Key, uint64_t> distances;
};
struct sqt_pair_heap_impl {
  using index_type = size_t;
  using distance_type = uint64_t;
  using element_type = std::pair<sqt, distance_type>;
  inline distance_type distance(element_type e) {
    return e.second;
  }
  inline sqt name(element_type e) {
    return e.first;
  }
};
struct sqt_heap_impl : sqt_pair_heap_impl {
  std::map<sqt, size_t> reverse;
  std::vector<std::pair<sqt, size_t*>> elements;
  inline element_type front(distance_type d) {
    assert(elements.size());
    return {elements.front().first, d};
  }
  inline void swap(index_type a, index_type b) {
    assert(*elements.at(a).second == a);
    assert(*elements.at(b).second == b);
    std::swap(elements.at(a), elements.at(b));
    std::swap(*elements.at(a).second, *elements.at(b).second);
    assert(*elements.at(a).second == a);
    assert(*elements.at(b).second == b);
  }
  inline void pop_back() {
    assert(elements.size());
    *elements.back().second = -1ULL;
    elements.pop_back();
  }
  inline void push_back(element_type e) {
    auto [it, inserted] = reverse.emplace(e.first, -1ULL);
    assert(it->second == -1ULL);
    it->second = elements.size();
    elements.push_back({e.first, std::addressof(it->second)});
  }
  inline sqt name_at(index_type i) {
    return name({elements.at(i).first, -1ULL});
  }
  inline index_type find_index(element_type e) {
    auto it = reverse.find(e.first);
    if (it == reverse.end()) {
      return -1ULL;
    } else {
      return it->second;
    }
  }
  inline index_type size() const {
    return elements.size();
  }
};

#endif // MIN_HEAP_HPP
