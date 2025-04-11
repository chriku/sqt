#include "sqt.glsl"
#include <format>

struct sqt {
  inline sqt() : data_(sqt_impl::sqt_new()) {}
  inline sqt(uint8_t major, std::initializer_list<size_t> minors) : data_(sqt_impl::sqt_new()) {
    sqt_impl::sqt_set_major(data_, major);
    for (auto m : minors) {
      data_ = sqt_impl::sqt_add_minor(data_, m);
    }
  }
  inline sqt(uint8_t major, auto minors) : data_(sqt_impl::sqt_new()) {
    sqt_impl::sqt_set_major(data_, major);
    for (auto m : minors) {
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
  auto operator<=>(const sqt&) const = default;
  std::array<sqt, 3> get_neighbors() const {
    std::array<sqt, 3> ret;
    sqt_impl::sqt_get_neighbors(data_, ret[0].data_, ret[1].data_, ret[2].data_);
    return ret;
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
    std::format_to(ctx.out(), "sqt_t({}", v.major());
    for (size_t i = 0; i < v.count(); i++) {
      std::format_to(ctx.out(), ", {}", v.minor(i));
    }
    return std::format_to(ctx.out(), ")");
  }
};
