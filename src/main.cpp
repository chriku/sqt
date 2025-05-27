#include "sqt.hpp"
#include "sqt_tree.hpp"

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest/doctest.h"

#include "min_heap.hpp"
#include "osm.hpp"
#include <etl/flat_map.h>
#include <etl/map.h>
#include <flat_set>
#include <generator>
#include <glm/gtx/string_cast.hpp>
#include <nlohmann/json.hpp>
#include <print>
#include <queue>
#include <random>
#include <ranges>
#include <set>
#include <thread>

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

std::array<glm::dvec2, 20> icostriangles = {{glm::dvec2(-DEG144, E_RAD),
    glm::dvec2(-DEG72, E_RAD),
    glm::dvec2(0.0, E_RAD),
    glm::dvec2(DEG72, E_RAD),
    glm::dvec2(DEG144, E_RAD),
    glm::dvec2(-DEG144, F_RAD),
    glm::dvec2(-DEG72, F_RAD),
    glm::dvec2(0.0, F_RAD),
    glm::dvec2(DEG72, F_RAD),
    glm::dvec2(DEG144, F_RAD),
    glm::dvec2(-DEG108, -F_RAD),
    glm::dvec2(-DEG36, -F_RAD),
    glm::dvec2(DEG36, -F_RAD),
    glm::dvec2(DEG108, -F_RAD),
    glm::dvec2(DEG180, -F_RAD),
    glm::dvec2(-DEG108, -E_RAD),
    glm::dvec2(-DEG36, -E_RAD),
    glm::dvec2(DEG36, -E_RAD),
    glm::dvec2(DEG108, -E_RAD),
    glm::dvec2(DEG180, -E_RAD)}};

std::array<glm::dvec2, 12> vertex = {{glm::dvec2(0.0, DEG90),
    glm::dvec2(DEG180, V_LAT),
    glm::dvec2(-DEG108, V_LAT),
    glm::dvec2(-DEG36, V_LAT),
    glm::dvec2(DEG36, V_LAT),
    glm::dvec2(DEG108, V_LAT),
    glm::dvec2(-DEG144, -V_LAT),
    glm::dvec2(-DEG72, -V_LAT),
    glm::dvec2(0.0, -V_LAT),
    glm::dvec2(DEG72, -V_LAT),
    glm::dvec2(DEG144, -V_LAT),
    glm::dvec2(0.0, -DEG90)}};

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
  for (auto p : s.get_raw_points_dvec3())
    stream << std::format("v {:.10f} {:.10f} {:.10f}\n", -p.x, -p.z, p.y);
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
      // if (tree[v] != tile::undefined)
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

//
inline glm::dvec2 uniform_random_point(auto& generator) {
  std::uniform_real_distribution<double> uniform01(0.0, 1.0);
  double theta = 2 * M_PI * uniform01(generator);
  double phi = acos(1 - 2 * uniform01(generator));
  return {theta, phi - (M_PI * 0.5)};
}

inline auto find_points_in_zone(const auto& points, sqt a) {
  return std::ranges::subrange(points.lower_bound(a), points.end()) |
         std::views::take_while([a](sqt b) { return a == b.counted(a.count()); });
}

inline nlohmann::json geojson(glm::dvec2 x) {
  while (x.x < -M_PI)
    x.x += M_PI * 2;
  while (x.x > M_PI)
    x.x -= M_PI * 2;
  return {x.x / M_PI * 180.0, (x.y / M_PI * 180.0)};
}
inline nlohmann::json geojson(sqt x) {
  return geojson(x.get_midpoint_latlond());
}

#if 1
TEST_CASE("acc1") {
  glm::dvec2 pt{3.15476763892379, -0.0015830220729089461};
  auto v = sqt(pt, 24);
  CHECK((sqt_impl::sqt_distance_latlond(pt, v.get_midpoint_latlond()) * 6000) < 20);
}
TEST_CASE("acc2") {
  glm::dvec2 pt{3.15476763892379, -0.0015830220729089461};
  auto v = sqt(glm::vec2(pt), 27);
  CHECK((sqt_impl::sqt_distance_latlond(pt, conv(v.get_midpoint_dvec3())) * 6000) < 20);
}
TEST_CASE("acc3") {
  glm::dvec2 pt{3.1242046036682742, -0.00610426997092528};
  auto v = sqt(pt, 24);
  std::println("FOUND {}", v);
  CHECK((sqt_impl::sqt_distance_latlond(pt, v.get_midpoint_latlond()) * 6000) < 20);
}
#endif

struct distance_pair_less {
  bool operator()(std::pair<sqt, sqt> a, std::pair<sqt, sqt> b) const {
    if (a.first < b.first)
      return true;
    if (a.first > b.first)
      return false;
    return a.second < b.second;
  }
};

using dist_map = etl::flat_map<uint32_t, uint64_t, 14>;
void route(min_heap<idx_heap_impl<4000000>, fixed_distance_container<4000000>>& frontier, std::vector<dist_map>& distances, uint32_t end) {
  while (frontier.impl_.size() >= 1) {
    assert(!frontier.empty());
    assert(frontier.impl_.size() >= 1);
    auto [node, dist] = frontier.top();
    frontier.pop();
    if (node == end)
      break;
    assert(node < distances.size());
    for (auto [nbr, len] : distances[node]) {
      size_t ndist = dist + len;
      frontier.push({nbr, ndist});
    }
  }
}

#if 1
TEST_CASE("") {
  std::ofstream file;
  file.exceptions(std::ios::badbit | std::ios::failbit);
  file.open("/home/christian/sqt/tree.obj", std::ios::trunc);

  sqt_tree<tile> tree(tile::undefined);
  read_osm(tree);
  {
    std::println("Filling water...");
    auto start = std::chrono::steady_clock::now();
    mark_coast(tree);
    std::println("Filling water... done in {}s",
        std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::steady_clock::now() - start).count());
  }
  std::flat_set<sqt> points;
  {
    std::println("Generating points...");
    auto start = std::chrono::steady_clock::now();
    constexpr size_t point_count = 4000000;
    std::vector<std::jthread> threads;
    std::mutex pts_mtx;
    std::set<sqt> pts;
    for (size_t ri = 0; ri < std::jthread::hardware_concurrency(); ri++)
      threads.emplace_back([&, ri] {
        std::set<sqt> p;
        std::mt19937 generator(ri);
        const size_t rmax = (((point_count / std::jthread::hardware_concurrency()) + 1) * 1.01);
        while (p.size() <= rmax) {
          glm::dvec2 pt = uniform_random_point(generator);
          auto v = sqt(pt, 27);
          auto fl = tree.first_leaf(v);
          assert(fl != nullptr);
          if (*fl == tile::water) {
            p.insert(v);
          }
        }
        std::unique_lock lock(pts_mtx);
        pts.merge(std::move(p));
      });
    for (auto& t : threads)
      t.join();
    std::mt19937 generator;
    while (pts.size() > point_count) {
      auto v = sqt(uniform_random_point(generator), 27);
      auto it = pts.lower_bound(v);
      if (it != pts.end())
        pts.erase(it);
    }
    points = {std::sorted_unique, pts.begin(), pts.end()};
    std::println("POINTS {} => in_water {}", point_count, points.size());
    std::println("Generating points... done in {}s",
        std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::steady_clock::now() - start).count());
  }
  std::map<sqt, uint32_t> reverse_points;
  using dist_map = etl::flat_map<uint32_t, uint64_t, 14>;
  std::vector<dist_map> distances;
  {
    std::println("Setup point structure... ({}B per distance)", sizeof(dist_map));
    auto start = std::chrono::steady_clock::now();
    for (auto [i, pt] : points | std::views::enumerate)
      reverse_points.emplace(pt, i);
    distances.resize(points.size());
    std::println("Setup point structure... done in {}s",
        std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::steady_clock::now() - start).count());
  }
  {
    std::println("Finding neighbors...");
    auto start = std::chrono::steady_clock::now();

    std::atomic<size_t> point_index = 0;
    std::vector<std::jthread> threads;
    std::array<std::mutex, 65537> distmtx;
    size_t maxnbr = 0;
    for (size_t i = 0; i < std::jthread::hardware_concurrency(); i++)
      threads.emplace_back([&] {
        size_t max = 0;
        while (true) {
          auto idx = point_index++;
          if (idx >= points.size())
            break;
          auto x = *(points.begin() + idx);
          sqt pp = x;
          sqt pn = x;
          sqt np = x;
          sqt nn = x;
          auto pos = x.get_midpoint_latlond();
          for (auto n : x.counted(7).get_self_and_neighbors())
            for (auto o : find_points_in_zone(points, n)) {
              auto ass = [&](auto& v) {
                if ((x == v) || (x.distance_dvec3(o) < x.distance_dvec3(v)))
                  v = o;
              };
              if (x == o)
                continue;
              auto diff = pos - o.get_midpoint_latlond();
              while (diff.x < -M_PI)
                diff.x += M_PI * 2;
              while (diff.x > M_PI)
                diff.x -= M_PI * 2;
              assert(fabs(diff.y) < M_PI);
              if (diff.x > 0) {
                if (diff.y > 0)
                  ass(pp);
                else
                  ass(pn);
              } else {
                if (diff.y > 0)
                  ass(np);
                else
                  ass(nn);
              }
            }
          for (auto o : {pp, pn, np, nn}) {
            uint64_t dist = x.distance_latlond(o) * 6371000000.0; // Earth diameter in mm
            if ((x != o) && (dist < 30000000)) {                  // 30km in mm
              assert(dist != 0);
              {
                std::unique_lock lock(distmtx.at(reverse_points.at(x) % distmtx.size()));
                distances.at(reverse_points.at(x))[reverse_points.at(o)] = dist;
                size_t v = distances.at(reverse_points.at(x)).size();
                if (v > max)
                  max = v;
              }
              {
                std::unique_lock lock(distmtx.at(reverse_points.at(o) % distmtx.size()));
                distances.at(reverse_points.at(o))[reverse_points.at(x)] = dist;
                size_t v = distances.at(reverse_points.at(o)).size();
                if (v > max)
                  max = v;
              }
            }
          }
        }
        std::unique_lock lock(distmtx.at(0));
        if (max > maxnbr)
          maxnbr = max;
      });
    for (auto& t : threads)
      t.join();

    std::println("Finding neighbors... done in {}s; max neighbors: {}",
        std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::steady_clock::now() - start).count(),
        maxnbr);
  }
#if 1
  {
    std::println("Finding simple routes...");
    auto start = std::chrono::steady_clock::now();

    size_t count = 100;
    size_t errors = 0;
    for (size_t i = 0; i < count; i++)
      try {
        auto startt = std::chrono::steady_clock::now();
        std::mt19937 generator(1748303721 + i);
        std::uniform_int_distribution<size_t> distr(0, points.size() - 1);
        auto start = *(points.begin() + distr(generator));
        auto end = reverse_points.at(*(points.begin() + distr(generator)));
        // start = *points.lower_bound(sqt({5.924440699059232 / 180.0 * sqt_impl::PI, 40.65103747458522 / 180.0 * sqt_impl::PI}, 27));
        // end = reverse_points.at(*points.lower_bound(sqt({0 / 180.0 * sqt_impl::PI, 0 / 180.0 * sqt_impl::PI}, 27)));
        // end = reverse_points.at(
        //     *points.lower_bound(sqt({34.25509008417936 / 180.0 * sqt_impl::PI, 43.291032400186936 / 180.0 * sqt_impl::PI}, 27)));
        if (start == *(points.begin() + end)) {
          throw std::runtime_error("No movement");
        }
        // std::unordered_map<sqt, uint64_t> routes;
        // routes[start] = 0;
        // std::unordered_map<sqt, sqt> prevs;
        using distpair = std::pair<uint64_t, sqt>;
        auto* frontier = new min_heap<idx_heap_impl<4000000>, fixed_distance_container<4000000>>();
        frontier->push({reverse_points.at(start), 0});
        auto startt2 = std::chrono::steady_clock::now();
        route(*frontier, distances, end);
        auto startt3 = std::chrono::steady_clock::now();
        std::vector<sqt> outpoints;
        outpoints.reserve(4000);
        {
          sqt cur = *(points.begin() + end);
          while (cur != start) {
            auto prev = [&] {
              auto cur_idx = reverse_points.at(cur);
              auto md = frontier->distance_for_element({cur_idx, -1ULL});
              for (auto [nbr, len] : distances.at(cur_idx)) {
                if ((frontier->distance_for_element({nbr, -1ULL}) + len) == md) {
                  cur = *(points.begin() + nbr);
                  return;
                }
              }
              for (auto [nbr, len] : distances.at(cur_idx)) {
                std::println("ERRORING {} + {} = {} != {}",
                    frontier->distance_for_element({nbr, -1ULL}),
                    len,
                    frontier->distance_for_element({nbr, -1ULL}) + len,
                    md);
              }
              throw std::runtime_error("No route");
            };
            prev();
            outpoints.push_back(cur);
          }
          assert(cur == start);
        }
        if (false) {
          nlohmann::json j = nlohmann::json::array();
          for (auto cur : outpoints)
            j.push_back(geojson(cur));
          std::println("{}", j.dump());
        }

        std::println("DJK (Seed {}), resulted in {} points {}s -> {}s",
            i,
            outpoints.size(),
            std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::steady_clock::now() - startt).count(),
            std::chrono::duration_cast<std::chrono::duration<double>>((std::chrono::steady_clock::now() - startt3) + (startt2 - startt))
                .count());
      } catch (const std::exception& e) {
        std::println("Error whle finding route: {}", e.what());
        errors++;
      }

    double t = std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::steady_clock::now() - start).count();
    std::println("Finding simple routes... done in {}s ({}s average), {} errors", t, t / count, errors);
  }
#endif
  /*{
    std::println("Writing .obj...");
    auto start2 = std::chrono::steady_clock::now();
    write_tree(tree);
    std::println("Writing .obj... done in {}s",
        std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::steady_clock::now() - start2).count());
  }*/
}
#endif
#if 1
TEST_CASE("") {
  sqt_tree<bool> tree(false);
  auto v = sqt{3, {0, 0, 0, 0, 0, 0, 0, 0, 0}};
  // auto v = sqt{3, {0, 1, 1, 1, 1, 1, 1, 1, 1}};
  // auto v = sqt{3, {2, 2, 2, 2, 2, 2, 2, 2, 2}};

  tree.set(sqt(19, {0, 2, 2, 1, 1, 1}), 43);
  tree.set(sqt(19, {3, 2, 2, 1, 1, 1}), 43);

  std::println(
      "DIVD A {} {}", sqt(19, {0, 2, 2, 1, 1, 1}).distance_dvec3(sqt(19, {0})), sqt(19, {0, 2, 2, 1, 1, 1}).distance_dvec3(sqt(19, {3})));
  std::println(
      "DIVD B {} {}", sqt(19, {3, 2, 2, 1, 1, 1}).distance_dvec3(sqt(19, {3})), sqt(19, {3, 2, 2, 1, 1, 1}).distance_dvec3(sqt(19, {0})));
  /*for (auto n : v.get_neighbors()) {
    std::println("SQT {} {}", v, n);
    tree.set(n, 43);
    for (auto n2 : n.get_neighbors())
      tree.set(n2, 43);
    for (auto n2 : n.get_neighbors())
      for (auto n3 : n2.get_neighbors())
        tree.set(n3, 43);
  }*/
  write_tree(tree);
}
#endif
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
            for (size_t e = 0; e < 4; e++) {
              co_yield sqt(major, {a, b, c, d, e});
              for (size_t f = 0; f < 4; f++) {
                co_yield sqt(major, {a, b, c, d, e, f});
              }
            }
          }
        }
      }
    }
    break;
  }
}

/*TEST_CASE("find_points_in_zone") {
  std::set<sqt> points;
  for (auto t : bunch_of_triangles())
    points.insert(t);
  for (auto t : bunch_of_triangles()) {
    std::set<sqt> v;
    for (auto p : find_points_in_zone(points, t))
      v.insert(p);
    std::set<sqt> w;
    for (auto p : points)
      if ((p.count() >= t.count()) && (t == p.counted(t.count())))
        w.insert(p);
    CHECK(v.size() == w.size());
    for (const auto& [a, b] : std::views::zip(v, w))
      CHECK(a == b);
  }
}*/

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
TEST_CASE("iterate2") {
  for (sqt v : bunch_of_triangles()) {
    for (size_t i = 0; i < v.count(); i++) {
      /*std::println("{}: {} == {} ({:#018x} == {:#018x})",
          v,
          v.counted(i),
          sqt(v.major(), v | std::views::take(i)),
          v.counted(i).data_._data,
          sqt(v.major(), v | std::views::take(i)).data_._data);*/
      CHECK(v.counted(i) == sqt(v.major(), v | std::views::take(i)));
    }
  }
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
    if (sqt(v.get_midpoint_dvec3(), v.count()) != v)
      std::println(
          "DIFF {} from {} at {}", sqt(v.get_midpoint_dvec3(), v.count()), v, sqt(v.get_midpoint_dvec3(), v.count()).distance_latlond(v));
    CHECK(sqt(v.get_midpoint_dvec3(), v.count()) == v);
  }
}
#if 1
TEST_CASE("distance2") {
  sqt v(14, {0, 2, 0, 0, 0, 0, 2, 1, 0, 2, 2, 0, 0, 2, 2, 2, 2, 0, 0, 2, 2, 2, 2, 0});
  {
    if (sqt(v.get_midpoint_dvec3(), v.count()) != v)
      std::println("DIFF {} from {} at {}", sqt(v.get_midpoint_dvec3(), v.count()), v, glm::to_string(v.get_midpoint_dvec3()));
    CHECK(sqt(v.get_midpoint_dvec3(), v.count()) == v);
  }
}
TEST_CASE("distance3") {
  sqt v(14, {0, 2, 0, 0, 0, 0, 2, 1, 0, 2, 2, 0, 0, 2, 2, 2, 2, 0, 0, 2, 2, 2, 2, 0});
  {
    if (sqt(v.get_midpoint_dvec3(), v.count()) != v)
      std::println("DIFF {} from {} at {}", sqt(v.get_midpoint_dvec3(), v.count()), v, glm::to_string(v.get_midpoint_dvec3()));
    CHECK(sqt(v.get_midpoint_dvec3(), v.count()) == v);
  }
}
TEST_CASE("distance4") {
  sqt v(14, {0, 1, 0, 0, 0, 0, 1, 1, 0, 2, 2, 0, 1, 1, 0, 0, 1, 1, 1, 2, 2, 2, 2, 1});
  {
    if (sqt(v.get_midpoint_dvec3(), v.count()) != v)
      std::println("DIFF {} from {} at {}", sqt(v.get_midpoint_dvec3(), v.count()), v, glm::to_string(v.get_midpoint_dvec3()));
    CHECK(sqt(v.get_midpoint_dvec3(), v.count()) == v);
  }
}
TEST_CASE("distance5") {
  sqt v(19, {3, 2, 2, 1, 1, 1});
  /*CHECK(v.distance_dvec3(sqt(19, {3})) < v.distance_dvec3(sqt(19, {0})));
  if (!((v.distance_dvec3(sqt(19, {3})) < v.distance_dvec3(sqt(19, {0}))))) {
    std::println("DIFF3 {} attracts to {} instead of {}",
        glm::to_string(v.get_raw_midpoint_dvec3()),
        glm::to_string(sqt(19, {0}).get_raw_midpoint_dvec3()),
        glm::to_string(sqt(19, {3}).get_raw_midpoint_dvec3()));

    std::ofstream file;
    file.exceptions(std::ios::badbit | std::ios::failbit);
    file.open("/home/christian/sqt/fuckfuck.obj", std::ios::trunc);
    write_face(file, v);
    /*write_face(file, sqt(19, {0}));
    write_face(file, sqt(19, {3}));*/
  //}
  auto np = sqt(v.get_midpoint_dvec3(), v.count());
  {
    if (np != v) {
      std::println("DIFF {} from {} at {}", np, v, np.distance_dvec3(v));
    }
    CHECK(np == v);
  }
}
#endif
#endif
