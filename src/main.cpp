#include "sqt.hpp"
#include "sqt_tree.hpp"

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest/doctest.h"

#include "osm.hpp"
#include <generator>
#include <glm/gtx/string_cast.hpp>
#include <print>
#include <ranges>
#include <set>

double DEG36 = 0.62831853071795864768;
double DEG72 = 1.25663706143591729537;
double DEG90 = M_PI / 2;
double DEG108 = 1.88495559215387594306;
double DEG120 = 2.09439510239319549229;
double DEG144 = 2.51327412287183459075;
double DEG180 = M_PI;

/* 26.565051177 degrees */
double V_LAT = 0.46364760899944494524;

/* 52.62263186 */
double E_RAD = 0.91843818702186776133;

/* 10.81231696 */
double F_RAD = 0.18871053072122403508;

using IseaGeo = glm::dvec2;
std::array<glm::dvec2, 20> icostriangles = {{IseaGeo(-DEG144, E_RAD),
    IseaGeo(-DEG72, E_RAD),
    IseaGeo(0.0, E_RAD),
    IseaGeo(DEG72, E_RAD),
    IseaGeo(DEG144, E_RAD),
    IseaGeo(-DEG144, F_RAD),
    IseaGeo(-DEG72, F_RAD),
    IseaGeo(0.0, F_RAD),
    IseaGeo(DEG72, F_RAD),
    IseaGeo(DEG144, F_RAD),
    IseaGeo(-DEG108, -F_RAD),
    IseaGeo(-DEG36, -F_RAD),
    IseaGeo(DEG36, -F_RAD),
    IseaGeo(DEG108, -F_RAD),
    IseaGeo(DEG180, -F_RAD),
    IseaGeo(-DEG108, -E_RAD),
    IseaGeo(-DEG36, -E_RAD),
    IseaGeo(DEG36, -E_RAD),
    IseaGeo(DEG108, -E_RAD),
    IseaGeo(DEG180, -E_RAD)}};

std::array<glm::dvec2, 12> vertex = {{IseaGeo(0.0, DEG90),
    IseaGeo(DEG180, V_LAT),
    IseaGeo(-DEG108, V_LAT),
    IseaGeo(-DEG36, V_LAT),
    IseaGeo(DEG36, V_LAT),
    IseaGeo(DEG108, V_LAT),
    IseaGeo(-DEG144, -V_LAT),
    IseaGeo(-DEG72, -V_LAT),
    IseaGeo(0.0, -V_LAT),
    IseaGeo(DEG72, -V_LAT),
    IseaGeo(DEG144, -V_LAT),
    IseaGeo(0.0, -DEG90)}};

glm::dvec3 conv(glm::dvec2 p) {
  p += glm::dvec2{M_PI, M_PI / 2};
  return {sin(p.y) * cos(p.x), sin(p.y) * sin(p.x), cos(p.y)};
}
glm::dvec2 conv(glm::dvec3 p) {
  return glm::dvec2{atan2(p.y, p.x), atan2(hypot(p.y, p.x), p.z)} - glm::dvec2{M_PI, M_PI / 2};
}

/*TEST_CASE("") {
  for (auto [i, p] : icostriangles | std::views::enumerate) {
    // std::println("{} to {}", glm::to_string(conv({0, 0})), glm::to_string(conv(conv({0, 0}))));
    // std::println("DIFF {}", glm::to_string(conv(p) - conv(conv(conv(p)))));
    std::array<size_t, 3> v = {{size_t(-1), size_t(-1), size_t(-1)}};
    for (auto [k, q] : vertex | std::views::enumerate) {
      auto dist = glm::length(conv(p) - conv(q));
      if (dist < 0.7) {
        for (auto [j, r] : sqt(i, {}).get_neighbors() | std::views::enumerate)
          if (glm::length(conv(q) - conv(icostriangles[r.major()])) > 1)
            v.at(j) = k;
      }
    }
    std::println("{{uint8_t({}), uint8_t({}), uint8_t({})}},", v.at(0), v.at(1), v.at(2));
  }
}*/

void write_face(auto& stream, sqt s) {
  static uint64_t cnt = 1;
  for (auto p : s.get_points_ndvec3())
    stream << std::format("v {:.10f} {:.10f} {:.10f}\n", p.x * 60, p.y * 60, p.z * 60);
  stream << std::format("f {} {} {}\n", cnt, cnt + 1, cnt + 2);
  cnt += 3;
}

/*TEST_CASE("") {
  sqt v(7, {});
  for (size_t i = 0; i < 10; i++) {
    v = v.add_minor(3);
    for (auto p : v.get_neighbors())
      write_face(p);
  }
  write_face(v);
}*/
void write_tree(auto& tree) {
  std::ofstream file;
  file.exceptions(std::ios::badbit | std::ios::failbit);
  file.open("/home/christian/sqt/tree.obj", std::ios::trunc);
  std::set<sqt> todo;
  for (size_t i = 0; i < 20; i++)
    todo.insert(sqt(i, {}));
  std::set<sqt> done;
  while (todo.size()) {
    auto it = todo.begin();
    auto v = *it;
    todo.erase(it);
    if (done.contains(v))
      continue;
    done.insert(v);
    if (tree.is_leaf(v)) {
      if (tree[v] != tile::undefined)
        write_face(file, v);
    } else {
      for (size_t i = 0; i < 4; i++)
        todo.insert(v.add_minor(i));
    }
  }
}
/*TEST_CASE("") {
  sqt_tree<int> tree(38);
  tree.set(sqt{3, {1}}, 42);
  tree.set(sqt{3, {0, 1, 1, 1, 1}}, 43);
}*/

TEST_CASE("") {
  sqt_tree<tile> tree(tile::undefined);
  read_osm(tree);
  {
    std::println("Filling water...");
    auto start = std::chrono::steady_clock::now();
    mark_coast(tree);
    std::println("Filling water... done in {}s",
        std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::steady_clock::now() - start).count());
  }
  {
    std::println("Writing .obj...");
    auto start2 = std::chrono::steady_clock::now();
    write_tree(tree);
    std::println("Writing .obj... done in {}s",
        std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::steady_clock::now() - start2).count());
  }
}
/*TEST_CASE("") {
  sqt_tree<bool> tree(false);
  auto v = sqt{3, {0, 0, 0, 0, 0, 0, 0, 0, 0}};
  // auto v = sqt{3, {0, 1, 1, 1, 1, 1, 1, 1, 1}};
  // auto v = sqt{3, {2, 2, 2, 2, 2, 2, 2, 2, 2}};

  tree.set(v, 43);
  for (auto n : v.get_neighbors()) {
    std::println("SQT {} {}", v, n);
    tree.set(n, 43);
    for (auto n2 : n.get_neighbors())
      tree.set(n2, 43);
    for (auto n2 : n.get_neighbors())
      for (auto n3 : n2.get_neighbors())
        tree.set(n3, 43);
  }
  write_tree(tree);
}*/
#if 1
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
TEST_CASE("distance") {
  for (sqt v : bunch_of_triangles()) {
    if (sqt(v.get_midpoint_ndvec3(), v.count()) != v)
      std::println("DIFF {} from {} at {}", sqt(v.get_midpoint_ndvec3(), v.count()), v, glm::to_string(v.get_midpoint_ndvec3()));
    CHECK(sqt(v.get_midpoint_ndvec3(), v.count()) == v);
  }
}
#endif