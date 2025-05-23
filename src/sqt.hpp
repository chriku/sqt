#ifndef SQT_HPP
#define SQT_HPP

namespace sqt_impl {
  struct sqt_t;
}
inline void dbgv(sqt_impl::sqt_t d);
#include <glm/glm.hpp>
template <typename T = double> inline glm::tvec3<T, glm::highp> conv(glm::tvec2<T, glm::highp> p) {
  p += glm::tvec2<T, glm::highp>{M_PI, M_PI / 2};
  return {sin(p.y) * cos(p.x), sin(p.y) * sin(p.x), cos(p.y)};
}
template <typename T = double> inline glm::tvec2<T, glm::highp> conv(glm::tvec3<T, glm::highp> rp) {
  auto p = glm::normalize(rp);
  return glm::dvec2{atan2(p.y, p.x), atan2(hypot(p.y, p.x), p.z)} - glm::dvec2{M_PI, M_PI / 2};
}

#include "sqt.glsl"
#include <format>
#include <iterator>
#include <ranges>

struct sqt {
  inline sqt() : data_(sqt_impl::sqt_new()) {}
  // inline sqt(glm::vec2 pos, sqt_impl::uint granularity) : data_(sqt_impl::sqt_from_point_latlonf({pos.x, pos.y}, granularity)) {}
  inline sqt(glm::dvec2 pos, sqt_impl::uint granularity) : data_(sqt_impl::sqt_from_point_dvec3(conv(pos), granularity)) {}
  // inline sqt(glm::vec3 pos, sqt_impl::uint granularity) : data_(sqt_impl::sqt_from_point_vec3(pos, granularity)) {}
  inline sqt(glm::dvec3 pos, sqt_impl::uint granularity) : data_(sqt_impl::sqt_from_point_dvec3(pos, granularity)) {}
  inline sqt(uint8_t major, std::initializer_list<size_t> minors) : data_(sqt_impl::sqt_new()) {
    data_ = sqt_impl::sqt_set_major(data_, major);
    for (auto m : minors) {
      assert(sqt_impl::sqt_count(data_) < sqt_impl::minor_count);
      data_ = sqt_impl::sqt_add_minor(data_, m);
    }
  }
  inline sqt(uint8_t major, auto minors) : data_(sqt_impl::sqt_new()) {
    data_ = sqt_impl::sqt_set_major(data_, major);
    for (auto m : minors) {
      assert(sqt_impl::sqt_count(data_) < sqt_impl::minor_count);
      data_ = sqt_impl::sqt_add_minor(data_, m);
    }
  }
  [[gnu::const]] inline uint32_t count() const {
    return sqt_impl::sqt_count(data_);
  }
  [[gnu::const]] inline uint32_t major() const {
    return sqt_impl::sqt_major(data_);
  }
  [[gnu::const]] inline uint32_t minor(uint32_t i) const {
    return sqt_impl::sqt_minor(data_, i);
  }
  inline void set_major(uint32_t v) {
    data_ = sqt_impl::sqt_set_major(data_, v);
  }
  [[gnu::const]] inline sqt add_minor(uint32_t v) const {
    return sqt_impl::sqt_add_minor(data_, v);
  }
  [[gnu::const]] inline sqt counted(uint32_t v) const {
    sqt d2 = *this;
    d2.data_ = sqt_impl::sqt_set_count(d2.data_, v);
    return d2;
  }
  auto operator<=>(const sqt&) const = default;
  [[gnu::const]] inline std::array<sqt, 3> get_neighbors() const {
    std::array<sqt, 3> ret;
    auto [n0, n1, n2] = sqt_impl::sqt_get_neighbors(data_);
    ret[0].data_ = n0;
    ret[1].data_ = n1;
    ret[2].data_ = n2;
    return ret;
  }
  [[gnu::const]] inline std::array<sqt, 4> get_self_and_neighbors() const {
    auto [n0, n1, n2] = get_neighbors();
    return {*this, n0, n1, n2};
  }
  [[gnu::const]] inline bool is_neighbor(sqt other) const {
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
  [[gnu::const]] inline iterator begin() const {
    return iterator{data_, 0};
  }
  [[gnu::const]] inline iterator end() const {
    return iterator{data_, uint8_t(sqt_impl::sqt_count(data_))};
  }
  /*[[gnu::const]] inline std::array<glm::vec3, 3> get_points_vec3() const {
    return sqt_impl::sqt_get_points_vec3(data_);
  }*/
  [[gnu::const]] inline std::array<glm::dvec3, 3> get_points_dvec3() const {
    return sqt_impl::sqt_get_points_dvec3(data_);
  }
  [[gnu::const]] inline std::array<glm::dvec3, 3> get_raw_points_dvec3() const {
    return sqt_impl::sqt_get_points_raw_dvec3(data_);
  }
  /*[[gnu::const]] inline glm::vec3 get_midpoint_vec3() const {
    return sqt_impl::sqt_get_midpoint_vec3(data_);
  }*/
  [[gnu::const]] inline glm::dvec3 get_midpoint_dvec3() const {
    return sqt_impl::sqt_get_midpoint_dvec3(data_);
  }
  [[gnu::const]] inline glm::dvec3 get_raw_midpoint_dvec3() const {
    return sqt_impl::sqt_get_raw_midpoint_dvec3(data_);
  }
  [[gnu::const]] inline glm::dvec2 get_midpoint_latlond() const {
    return conv(get_midpoint_dvec3());
  }
  /*[[gnu::const]] inline double distance_vec3(sqt other) const {
    return sqt_impl::sqt_distance_vec3(sqt_impl::sqt_get_midpoint_vec3(data_), sqt_impl::sqt_get_midpoint_vec3(other.data_));
  }*/
  [[gnu::const]] inline double distance_dvec3(sqt other) const {
    return sqt_impl::sqt_distance_dvec3(sqt_impl::sqt_get_raw_midpoint_dvec3(data_), sqt_impl::sqt_get_raw_midpoint_dvec3(other.data_));
  }
  [[gnu::const]] inline double distance_latlond(sqt other) const {
    return sqt_impl::sqt_distance_latlond(get_midpoint_latlond(), other.get_midpoint_latlond());
  }

  // private:
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

  inline auto format(sqt v, std::format_context& ctx) const {
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

inline std::ostream& operator<<(std::ostream& os, const sqt& value) {
  os << std::format("{}", value);
  return os;
}
inline void dbgv(sqt_impl::sqt_t d) {
  sqt s;
  s.data_ = d;
  // std::println("DBGV {}", s);
}

#endif // SQT_HPP
