#include "sqt.glsl"

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest/doctest.h"

#include <generator>
#include <print>
#include <ranges>
#include <set>

template <> struct std::formatter<sqt::sqt_t> {
  constexpr auto parse(std::format_parse_context& ctx) {
    auto it = ctx.begin();
    if (it == ctx.end())
      return it;
    if (*it != '}')
      throw std::format_error("Invalid format args for sqt::sqt_t.");
    return it;
  }

  auto format(sqt::sqt_t v, std::format_context& ctx) const {
    std::format_to(ctx.out(), "sqt_t({:#x}: {}", v._data, sqt::sqt_major(v));
    for (size_t i = 0; i < sqt::sqt_count(v); i++) {
      std::format_to(ctx.out(), ", {}", sqt::sqt_minor(v, i));
    }
    return std::format_to(ctx.out(), ")");
  }
};

std::generator<sqt::sqt_t> bunch_of_triangles() {
  for (size_t major = 0; major < 20; major++) {
    co_yield sqt::sqt_t(major, {});
    for (size_t a = 0; a < 4; a++) {
      co_yield sqt::sqt_t(major, {a});
      for (size_t b = 0; b < 4; b++) {
        co_yield sqt::sqt_t(major, {a, b});
        for (size_t c = 0; c < 4; c++) {
          co_yield sqt::sqt_t(major, {a, b, c});
          for (size_t d = 0; d < 4; d++) {
            co_yield sqt::sqt_t(major, {a, b, c, d});
          }
        }
      }
    }
  }
}

TEST_CASE("basic") {
  sqt::sqt_t a = sqt::sqt_new();
  CHECK(a._data == 0);
  CHECK(sqt::sqt_major(a) == 0);
  CHECK(sqt::sqt_count(a) == 0);
  sqt::sqt_set_major(a, 2);
  CHECK(a._data != 0);
  CHECK(sqt::sqt_major(a) == 2);
  CHECK(sqt::sqt_count(a) == 0);
  a = sqt::sqt_add_minor(a, 3);
  CHECK(sqt::sqt_major(a) == 2);
  CHECK(sqt::sqt_count(a) == 1);
  CHECK(sqt::sqt_minor(a, 0) == 3);
  a = sqt::sqt_add_minor(a, 0);
  CHECK(sqt::sqt_major(a) == 2);
  CHECK(sqt::sqt_count(a) == 2);
  CHECK(sqt::sqt_minor(a, 0) == 3);
  CHECK(sqt::sqt_minor(a, 1) == 0);
  a = sqt::sqt_add_minor(a, 1);
  CHECK(sqt::sqt_major(a) == 2);
  CHECK(sqt::sqt_count(a) == 3);
  CHECK(sqt::sqt_minor(a, 0) == 3);
  CHECK(sqt::sqt_minor(a, 1) == 0);
  CHECK(sqt::sqt_minor(a, 2) == 1);
  sqt::sqt_t b = {2, {3, 0, 1}};
  CHECK(a == b);
}
TEST_CASE("neighbors_self") {
  for (sqt::sqt_t v : bunch_of_triangles()) {
    sqt::sqt_t n0 = sqt::sqt_new();
    sqt::sqt_t n1 = sqt::sqt_new();
    sqt::sqt_t n2 = sqt::sqt_new();
    sqt::sqt_t nb0 = sqt::sqt_new();
    sqt::sqt_t nb1 = sqt::sqt_new();
    sqt::sqt_t nb2 = sqt::sqt_new();
    std::multiset<sqt::sqt_t> s;
    sqt_get_neighbors(v, n0, n1, n2);
    std::println("N {}: {} {} {}", v, n0, n1, n2);
    sqt_get_neighbors(n0, nb0, nb1, nb2);
    s.insert(nb0);
    s.insert(nb1);
    s.insert(nb2);
    sqt_get_neighbors(n1, nb0, nb1, nb2);
    s.insert(nb0);
    s.insert(nb1);
    s.insert(nb2);
    sqt_get_neighbors(n2, nb0, nb1, nb2);
    s.insert(nb0);
    s.insert(nb1);
    s.insert(nb2);
    CHECK(s.size() == 9);
    CHECK(s.count(v) == 3);
  }
}