#ifndef SQT_HPP
#define SQT_HPP
#include "sqt.glsl"
#include <format>
#include <iterator>
#include <ranges>

struct sqt {
  inline sqt() : data_(sqt_impl::sqt_new()) {}
  inline sqt(glm::dvec3 pos, sqt_impl::uint granularity) : data_(sqt_impl::sqt_from_point_ndvec3(pos, granularity)) {}
  inline sqt(uint8_t major, std::initializer_list<size_t> minors) : data_(sqt_impl::sqt_new()) {
    sqt_impl::sqt_set_major(data_, major);
    for (auto m : minors) {
      assert(sqt_impl::sqt_count(data_) < sqt_impl::minor_count);
      data_ = sqt_impl::sqt_add_minor(data_, m);
    }
  }
  inline sqt(uint8_t major, auto minors) : data_(sqt_impl::sqt_new()) {
    sqt_impl::sqt_set_major(data_, major);
    for (auto m : minors) {
      assert(sqt_impl::sqt_count(data_) < sqt_impl::minor_count);
      data_ = sqt_impl::sqt_add_minor(data_, m);
    }
  }
  uint32_t count() const {
    return sqt_impl::sqt_count(data_);
  }
  uint32_t major() const {
    return sqt_impl::sqt_major(data_);
  }
  uint32_t minor(uint32_t i) const {
    return sqt_impl::sqt_minor(data_, i);
  }
  void set_major(uint32_t v) {
    return sqt_impl::sqt_set_major(data_, v);
  }
  sqt add_minor(uint32_t v) const {
    return sqt_impl::sqt_add_minor(data_, v);
  }
  sqt counted(uint32_t v) const {
    sqt d2 = *this;
    sqt_impl::sqt_set_count(d2.data_, v);
    return d2;
  }
  auto operator<=>(const sqt&) const = default;
  std::array<sqt, 3> get_neighbors() const {
    std::array<sqt, 3> ret;
    sqt_impl::sqt_get_neighbors(data_, ret[0].data_, ret[1].data_, ret[2].data_);
    return ret;
  }
  bool is_neighbor(sqt other) const {
    assert(other.count() >= count());
    auto [a, b, c] = other.get_neighbors();
    if (*this == a.counted(count()))
      return true;
    if (*this == b.counted(count()))
      return true;
    if (*this == c.counted(count()))
      return true;
    return false;
  }

  struct iterator {
    using iterator_category = std::random_access_iterator_tag;
    using difference_type = int8_t;
    using value_type = uint8_t;
    sqt_impl::sqt_t data_ = sqt_impl::sqt_new();
    uint8_t index = 0;
    inline value_type operator*() const {
      return sqt_impl::sqt_minor(data_, index);
    }
    inline value_type operator[](difference_type d) const {
      return *((*this) + d);
    }
    inline difference_type operator-(iterator d) const {
      assert(data_ == d.data_);
      return index - d.index;
    }
    inline iterator operator+(difference_type d) const {
      iterator ret = *this;
      assert((long(ret.index) + long(d)) >= 0);
      ret.index += d;
      assert(ret.index <= sqt_impl::sqt_count(data_));
      return ret;
    }
    friend inline iterator operator+(difference_type d, iterator it) {
      return it + d;
    }
    inline iterator operator-(difference_type d) const {
      return (*this) + -d;
    }
    inline iterator& operator+=(difference_type d) {
      *this = (*this) + d;
      return *this;
    }
    inline iterator& operator-=(difference_type d) {
      *this = (*this) - d;
      return *this;
    }
    inline iterator& operator++() {
      index += 1;
      assert(index <= sqt_impl::sqt_count(data_));
      return *this;
    }
    inline iterator& operator--() {
      assert(index > 0);
      index -= 1;
      return *this;
    }
    inline iterator operator++(int) {
      iterator retval = *this;
      ++(*this);
      return retval;
    }
    inline iterator operator--(int) {
      iterator retval = *this;
      --(*this);
      return retval;
    }
    friend constexpr std::partial_ordering operator<=>(iterator a, iterator b) {
      std::partial_ordering v = a.data_ <=> b.data_;
      if ((v == std::partial_ordering::less) || (v == std::partial_ordering::greater))
        return v;
      return a.index <=> b.index;
    }
    constexpr bool operator==(iterator o) const {
      return ((*this) <=> o) == std::strong_ordering::equal;
    }
  };
  inline iterator begin() const {
    return iterator{data_, 0};
  }
  inline iterator end() const {
    return iterator{data_, uint8_t(sqt_impl::sqt_count(data_))};
  }
  std::array<glm::vec3, 3> get_points_vec3() const {
    return sqt_impl::sqt_get_points_vec3(data_);
  }
  std::array<glm::vec3, 3> get_points_nvec3() const {
    return sqt_impl::sqt_get_points_nvec3(data_);
  }
  std::array<glm::dvec3, 3> get_points_ndvec3() const {
    return sqt_impl::sqt_get_points_ndvec3(data_);
  }
  glm::vec3 get_midpoint_nvec3() const {
    std::array<glm::vec3, 3> ret;
    return sqt_impl::sqt_get_midpoint_nvec3(data_);
  }
  double distance_ndvec3(sqt other) const {
    return glm::length(sqt_impl::sqt_get_midpoint_ndvec3(data_) - sqt_impl::sqt_get_midpoint_ndvec3(other.data_));
  }

private:
  inline sqt(sqt_impl::sqt_t v) : data_(v) {}
  sqt_impl::sqt_t data_;
};

template <> struct std::formatter<sqt> {
  constexpr auto parse(std::format_parse_context& ctx) {
    auto it = ctx.begin();
    if (it == ctx.end())
      return it;
    if (*it != '}')
      throw std::format_error("Invalid format args for sqt.");
    return it;
  }

  auto format(sqt v, std::format_context& ctx) const {
    std::format_to(ctx.out(), "sqt_t({}, {{", v.major());
    for (size_t i = 0; i < v.count(); i++) {
      if (i)
        std::format_to(ctx.out(), ", {}", v.minor(i));
      else
        std::format_to(ctx.out(), "{}", v.minor(i));
    }
    return std::format_to(ctx.out(), "}})");
  }
};

#endif // SQT_HPP
