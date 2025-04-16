#include "sqt.hpp"

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest/doctest.h"

#include <generator>
#include <print>
#include <ranges>
#include <set>

std::generator<sqt> bunch_of_triangles() {
  for (size_t major = 0; major < 20; major++) {
    co_yield sqt(major, {});
    for (size_t a = 0; a < 4; a++) {
      co_yield sqt(major, {a});
      for (size_t b = 0; b < 4; b++) {
        co_yield sqt(major, {a, b});
        for (size_t c = 0; c < 4; c++) {
          co_yield sqt(major, {a, b, c});
          for (size_t d = 0; d < 4; d++) {
            co_yield sqt(major, {a, b, c, d});
          }
        }
      }
    }
  }
}

TEST_CASE("basic") {
  sqt a;
  CHECK(a.major() == 0);
  CHECK(a.count() == 0);
  a.set_major(2);
  CHECK(a.major() == 2);
  CHECK(a.count() == 0);
  a = a.add_minor(3);
  CHECK(a.major() == 2);
  CHECK(a.count() == 1);
  CHECK(a.minor(0) == 3);
  a = a.add_minor(0);
  CHECK(a.major() == 2);
  CHECK(a.count() == 2);
  CHECK(a.minor(0) == 3);
  CHECK(a.minor(1) == 0);
  a = a.add_minor(1);
  CHECK(a.major() == 2);
  CHECK(a.count() == 3);
  CHECK(a.minor(0) == 3);
  CHECK(a.minor(1) == 0);
  CHECK(a.minor(2) == 1);
  sqt b = {2, {3, 0, 1}};
  CHECK(a == b);
}
TEST_CASE("neighbors_self") {
  for (sqt v : bunch_of_triangles()) {
    sqt nb0, nb1, nb2;
    std::multiset<sqt> s;
    auto [n0, n1, n2] = v.get_neighbors();
    // std::println("N {}: {} {} {}", v, n0, n1, n2);
    auto [nba0, nba1, nba2] = n0.get_neighbors();
    s.insert(nba0);
    s.insert(nba1);
    s.insert(nba2);
    auto [nbb0, nbb1, nbb2] = n1.get_neighbors();
    s.insert(nbb0);
    s.insert(nbb1);
    s.insert(nbb2);
    auto [nbc0, nbc1, nbc2] = n2.get_neighbors();
    s.insert(nbc0);
    s.insert(nbc1);
    s.insert(nbc2);
    CHECK(s.size() == 9);
    CHECK(s.count(v) == 3);
  }
}
static_assert(std::input_or_output_iterator<sqt::iterator>);
static_assert(std::input_iterator<sqt::iterator>);
static_assert(std::forward_iterator<sqt::iterator>);
static_assert(std::bidirectional_iterator<sqt::iterator>);
static_assert(std::random_access_iterator<sqt::iterator>);
TEST_CASE("iterate") {
  sqt v = {10, {3, 0, 1}};
  std::vector<size_t> res;
  for (auto x : v) {
    res.push_back(x);
  }
  CHECK(res.size() == 3);
  CHECK(res.at(0) == 3);
  CHECK(res.at(1) == 0);
  CHECK(res.at(2) == 1);
}
TEST_CASE("iterate2") {
  sqt v = {10, {3, 0, 1}};
  std::vector<size_t> res;
  for (auto x : v | std::views::take(2)) {
    res.push_back(x);
  }
  CHECK(res.size() == 2);
  CHECK(res.at(0) == 3);
  CHECK(res.at(1) == 0);
}
TEST_CASE("neighbors") {
  sqt_tree<int> tree(38);
  tree.set(sqt{3, {1}}, 42);
  tree.set(sqt{3, {0, 1, 1, 1, 1}}, 43);
  std::vector<sqt> out;
  tree.all_neighbors(sqt{3, {3}}, std::back_inserter(out));
  for (auto n : out)
    std::println("NBR {} = {}", n, tree[n]);
}
