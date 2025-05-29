#include "sqt.hpp"
#include "sqt_tree.hpp"

#define DOCTEST_CONFIG_IMPLEMENT
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
#include <restinio/all.hpp>
#include <set>
#include <thread>

using namespace std::literals;

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

inline glm::dvec2 degreefy(glm::dvec2 x) {
  while (x.x < -M_PI)
    x.x += M_PI * 2;
  while (x.x > M_PI)
    x.x -= M_PI * 2;
  return {x.x / M_PI * 180.0, (x.y / M_PI * 180.0)};
}
inline nlohmann::json geojson(glm::dvec2 x) {
  return {degreefy(x).x, degreefy(x).y};
}
inline nlohmann::json geojson(sqt x) {
  return geojson(x.get_midpoint_latlond());
}

using dist_type = uint32_t;
using dist_map = etl::flat_map<uint32_t, dist_type, 14>;

struct router {
  virtual void route(sqt start, uint32_t end, std::vector<sqt>& outpoints, dist_type& outdist) = 0;
};
template <bool frontier_mode> struct simple_dijkstra : router {
  std::flat_set<sqt>& points;
  std::map<sqt, uint32_t>& reverse_points;
  std::vector<dist_map>& distances;
  using frontier_type = min_heap<idx_heap_impl<4000000, dist_type>, fixed_distance_container<4000000, dist_type>>;
  std::unique_ptr<frontier_type> frontier;
  inline simple_dijkstra(std::flat_set<sqt>& points, std::map<sqt, uint32_t>& reverse_points, std::vector<dist_map>& distances)
      : points(points), reverse_points(reverse_points), distances(distances) {
    frontier = std::make_unique<frontier_type>();
  }
  std::chrono::steady_clock::duration rtd = {};
  void route(sqt start, uint32_t end, std::vector<sqt>& outpoints, dist_type& outdist) override {
    if (start == *(points.begin() + end)) {
      throw std::runtime_error("No movement");
    }
    frontier->reset();
    auto start_idx = reverse_points.at(start);
    frontier->push({start_idx, 0});

    auto startt2 = std::chrono::steady_clock::now();

    auto resolve_route = [&] {
      auto cur_idx = end;
      while (cur_idx != start_idx) {
        auto prev = [&] {
          auto md = frontier->distance_for_element({cur_idx, -1ULL});
          for (auto& [nbr, len] : distances.at(cur_idx)) {
            if ((frontier->distance_for_element({nbr, -1ULL}) + len) == md) {
              cur_idx = nbr;
              outdist += len;
              return;
            }
          }
          for (auto& [nbr, len] : distances.at(cur_idx)) {
            std::println("ERRORING {} + {} = {} != {}",
                frontier->distance_for_element({nbr, -1ULL}),
                len,
                frontier->distance_for_element({nbr, -1ULL}) + len,
                md);
          }
          throw std::runtime_error("IMPL error");
        };
        prev();
        outpoints.push_back(*(points.begin() + cur_idx));
      }
      if (cur_idx != start_idx)
        throw std::runtime_error("IRE");
    };

    while (!frontier->empty()) {
      assert(!frontier->empty());
      assert(frontier->impl_.size() >= 1);
      auto [node, dist] = frontier->top();
      frontier->pop();
      if (node == end) [[unlikely]] {
        rtd += (std::chrono::steady_clock::now() - startt2);
        return resolve_route();
        break;
      }
      assert(node < distances.size());
      for (auto& [nbr, len] : distances[node]) {
        size_t ndist = dist + len;
        frontier->push({nbr, ndist});
      }
    }
    throw std::runtime_error("No route");
  }
};

struct projection {
  virtual bool is_mercator() {
    return false;
  }
  virtual bool consider(sqt) = 0;
  virtual glm::dvec2 project(glm::dvec2) = 0;
  inline glm::dvec2 project(sqt s) {
    return project(s.get_midpoint_latlond());
  }
};
struct mercator : projection {
  bool is_mercator() override {
    return true;
  }
  bool consider(sqt s) override {
    return true;
  }
  glm::dvec2 project(glm::dvec2 ll) override {
    while (ll.x < -M_PI)
      ll.x += M_PI * 2;
    while (ll.x > M_PI)
      ll.x -= M_PI * 2;
    return {((ll.x / M_PI) + 1) * 500, ((-ll.y / M_PI) + 0.5) * 500};
  }
  inline glm::dvec2 project(sqt s) {
    return project(s.get_midpoint_latlond());
  }
};
struct zoomed_mercator : projection {
  sqt center;
  double zoff = 2;
  double zoom;
  zoomed_mercator(sqt center, double z) : center(center), zoom(z - zoff) {}
  bool is_mercator() override {
    return true;
  }
  bool consider(sqt s) override {
    if (zoom < 0)
      return true;
    int64_t m = ceil(zoom);
    if (m > s.count())
      m = s.count();
    auto a = s.counted(m);
    auto b = center.counted(m);
    return (a == b) || (a.is_neighbor(b));
  }
  glm::dvec2 project(glm::dvec2 ll) override {
    ll -= center.get_midpoint_latlond() * fmin(1 + (zoom / zoff), 1);
    while (ll.x < -M_PI)
      ll.x += M_PI * 2;
    while (ll.x > M_PI)
      ll.x -= M_PI * 2;
    ll *= pow(2, zoom) * zoff * zoff;
    return {((ll.x / M_PI) + 1) * 500, ((-ll.y / M_PI) + 0.5) * 500};
  }
  inline glm::dvec2 project(sqt s) {
    return project(s.get_midpoint_latlond());
  }
};

int main(int argc, char** argv) {
  auto start = std::chrono::steady_clock::now();
  doctest::Context context;
  context.applyCommandLine(argc, argv);
  int res = context.run();  // run
  if (context.shouldExit()) // important - query flags (and --exit) rely on the user doing this
    return res;             // propagate the result of the tests

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
    std::println("Generating points... done in {}s ({} points)",
        std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::steady_clock::now() - start).count(),
        points.size());
  }
  std::map<sqt, uint32_t> reverse_points;
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
            dist_type dist = x.distance_latlond(o) * (6371000000.0 / 25); // Earth diameter in mm
            if ((x != o) && (dist < (30000000 / 25))) {                   // 30km in mm
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
  auto run = [&]<typename T>(T) {
    std::println("Finding simple routes... {}", T::value);
    auto start = std::chrono::steady_clock::now();

    constexpr size_t count = 1000;
    size_t errors = 0;
    simple_dijkstra<T::value> djk(points, reverse_points, distances);
    std::chrono::steady_clock::duration rtd = {};
    std::vector<sqt> outpoints;
    outpoints.reserve(4000 * count);
    for (size_t i = 0; i < count; i++)
      try {
        auto startt = std::chrono::steady_clock::now();
        std::mt19937 generator(1748303721 + i);
        std::uniform_int_distribution<size_t> distr(0, points.size() - 1);
        auto start = *(points.begin() + distr(generator));
        auto end = reverse_points.at(*(points.begin() + distr(generator)));
        dist_type outdist;
        auto startt2 = std::chrono::steady_clock::now();
        djk.route(start, end, outpoints, outdist);

        rtd += (std::chrono::steady_clock::now() - startt2);
      } catch (const std::exception& e) {
        std::println("Error whle finding route: {}", e.what());
        errors++;
      }

    double t = std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::steady_clock::now() - start).count();
    double t2 = std::chrono::duration_cast<std::chrono::duration<double>>(rtd).count();
    double t3 = std::chrono::duration_cast<std::chrono::duration<double>>(djk.rtd).count();
    std::println("Finding simple routes... done in {}s ({}s average), calc {}s ({}s average), loop {}s ({}s average), {} errors, {} points",
        t,
        t / count,
        t2,
        t2 / count,
        t3,
        t3 / count,
        errors,
        outpoints.size());
  };
  for (size_t i = 0; i < 1; i++) {
    run(std::false_type{});
  }

  std::println("Starting webserver on 127.0.0.1:8080 after {}s",
      std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::steady_clock::now() - start).count());

  std::mt19937 generator(1748303721);
  restinio::run(restinio::on_this_thread().port(8080).address("0.0.0.0").request_handler([&](auto req) {
    if ((req->header().path() != "/"sv) && (req->header().path() != "/graph.svg"sv))
      return req->create_response(restinio::status_not_found()).connection_close().done();
    const restinio::query_string_params_t qp = restinio::parse_query(req->header().query());
    std::stringstream svg;
    std::stringstream form;
    std::stringstream measures;
    std::unique_ptr<projection> proj;
    double zoom = 0;
    {
      zoom = opt_value<double>(qp, "zoom").value_or(0);
      sqt center = sqt(glm::dvec3(conv({-31.105278 / 180.0 * sqt_impl::PI, 39.6975 / 180.0 * sqt_impl::PI})), 27);
      auto center_lon = opt_value<double>(qp, "center_lon");
      auto center_lat = opt_value<double>(qp, "center_lat");
      if (center_lon && (center_lon.value() >= -180) && (center_lon.value() <= 180) && center_lat && (center_lat.value() >= -90) &&
          (center_lat.value() <= 90))
        center = sqt({center_lon.value() / 180.0 * sqt_impl::PI, center_lat.value() / 180.0 * sqt_impl::PI}, 27);
      form << std::format(
          R"(<tr><td><label for="zoom">Zoom (may truncate image early)</label><td><input type="number" name="zoom" value="{}" min="0" max="30" step="any"/>)",
          zoom);
      form << std::format(
          R"(<tr><td><label for="center_lat">Center Latitude</label><td><input type="number" name="center_lat" value="{}" min="-90" max="90" step="any"/>N)",
          degreefy(center.get_midpoint_latlond()).y);
      form << std::format(
          R"(<tr><td><label for="center_lon">Center Longitude</label><td><input type="number" name="center_lon" value="{}" min="-180" max="180" step="any"/>E)",
          degreefy(center.get_midpoint_latlond()).x);
      proj = std::make_unique<zoomed_mercator>(center, zoom);
    }
    if (!proj)
      proj = std::make_unique<mercator>();
    // bool fisch = opt_value<int>(qp, "fisch").value_or(0);
    /*bool fisch = qp.has("fisch");
    std::println("Fisch {}", fisch);*/
    /*form << std::format(R"(<tr><td><label for="mercator">Mercator</label><td><input type="radio" name="mercator" {}/>)",
        proj->is_mercator() ? "checked"sv : ""sv);*/
    std::uniform_int_distribution<size_t> distr(0, points.size() - 1);
    auto start = *(points.begin() + distr(generator));
    auto end = *(points.begin() + distr(generator));
    {
      auto start_lon = opt_value<double>(qp, "start_lon");
      auto start_lat = opt_value<double>(qp, "start_lat");
      if (start_lon && (start_lon.value() >= -180) && (start_lon.value() <= 180) && start_lat && (start_lat.value() >= -90) &&
          (start_lat.value() <= 90))
        start = *points.lower_bound(sqt({start_lon.value() / 180.0 * sqt_impl::PI, start_lat.value() / 180.0 * sqt_impl::PI}, 27));
      form << "<tr><td><td>Warning: Snaps to nearest point";
      form << std::format(
          R"(<tr><td><label for="start_lat">Start Latitude</label><td><input type="number" name="start_lat" value="{}" min="-90" max="90" step="any"/>N)",
          degreefy(start.get_midpoint_latlond()).y);
      form << std::format(
          R"(<tr><td><label for="start_lon">Start Longitude</label><td><input type="number" name="start_lon" value="{}" min="-180" max="180" step="any"/>E)",
          degreefy(start.get_midpoint_latlond()).x);
    }
    {
      auto end_lon = opt_value<double>(qp, "end_lon");
      auto end_lat = opt_value<double>(qp, "end_lat");
      if (end_lon && (end_lon.value() >= -180) && (end_lon.value() <= 180) && end_lat && (end_lat.value() >= -90) &&
          (end_lat.value() <= 90))
        end = *points.lower_bound(sqt({end_lon.value() / 180.0 * sqt_impl::PI, end_lat.value() / 180.0 * sqt_impl::PI}, 27));
      form << std::format(
          R"(<tr><td><label for="end_lat">End Latitude</label><td><input type="number" name="end_lat" value="{}" min="-90" max="90" step="any"/>N)",
          degreefy(end.get_midpoint_latlond()).y);
      form << std::format(
          R"(<tr><td><label for="end_lon">End Longitude</label><td><input type="number" name="end_lon" value="{}" min="-180" max="180" step="any"/>E)",
          degreefy(end.get_midpoint_latlond()).x);
    }
    {
      auto coast_depth = opt_value<uint32_t>(qp, "coast_depth").value_or(100);
      form << std::format(
          R"(<tr><td><label for="coast_depth">Draw Coast or Cut Triangles (0 to disable)</label><td><input type="number" name="coast_depth" value="{}" min="0"/>)",
          coast_depth);
      if (coast_depth > (zoom + 8))
        coast_depth = (zoom + 8);
      if (coast_depth) {
        svg << "<g>";
        std::function<void(sqt)> do_coast = [&](sqt s) {
          if (!proj->consider(s))
            return;
          if ((s.count() < coast_depth) && (!tree.touches_leaf(s))) {
            for (size_t i = 0; i < 4; i++)
              do_coast(s.add_minor(i));
          } else {
            auto l = tree.first_leaf(s);
            if ((!l) || (*l == tile::coast)) {
              auto outpoints = s.get_points_dvec3();
              if ((fabs(degreefy(conv(outpoints.at(0))).x - degreefy(conv(outpoints.at(1))).x) > 180) ||
                  (fabs(degreefy(conv(outpoints.at(0))).x - degreefy(conv(outpoints.at(2))).x) > 180) ||
                  (fabs(degreefy(conv(outpoints.at(1))).x - degreefy(conv(outpoints.at(2))).x) > 180))
                return;
              svg << "<path d=\"";
              if (outpoints.size() > 0) {
                auto p = proj->project(conv(outpoints.at(0)));
                svg << "M" << p.x << " " << p.y << " ";
              }
              for (auto s : outpoints | std::views::drop(1)) {
                auto p = proj->project(conv(s));
                svg << "L" << p.x << " " << p.y << " ";
              }
              svg << R"(z" style="stroke: black; fill: black; stroke-width: 0.01" />)";
            }
          }
        };
        for (size_t i = 0; i < 20; i++)
          do_coast(sqt(i, {}));
        svg << "</g>";
      }
    }
    {
      svg << "<g>";
      bool render_graph = qp.has("render_graph");
      form << std::format(
          R"(<tr><td><label for="render_graph">Render Full Graph (will annihilate your browser without zoom)</label><td><input type="checkbox" name="render_graph" {}/>)",
          render_graph ? "checked"sv : ""sv);
      if (render_graph) {
        for (const auto& [i, map] : distances | std::views::enumerate) {
          auto self = *(points.begin() + i);
          if (proj->consider(self))
            for (auto& [oi, _] : map) {
              auto other = *(points.begin() + oi);
              if (fabs(degreefy(self.get_midpoint_latlond()).x - degreefy(other.get_midpoint_latlond()).x) > 180)
                continue;
              svg << "<path d=\"";
              {
                auto p = proj->project(self);
                svg << "M" << p.x << " " << p.y << " ";
              }
              {
                auto p = proj->project(other);
                svg << "L" << p.x << " " << p.y << " ";
              }
              svg << R"(" style="stroke: blue; fill: none; stroke-width: 0.1" />)";
            }
        }
      }
      svg << "</g>";
    }
    {
      std::vector<sqt> outpoints;
      outpoints.reserve(8000);
      auto start_time = std::chrono::steady_clock::now();
      simple_dijkstra<true> djk(points, reverse_points, distances);
      dist_type outdist = 0;
      djk.route(start, reverse_points.at(end), outpoints, outdist);
      double t =
          std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(std::chrono::steady_clock::now() - start_time).count();
      measures << std::format("<p>Single Route Distance: {}km</p>", double(outdist) / 1000000ULL);
      measures << std::format("<p>Single Route Distance: {}mm</p>", outdist);
      measures << std::format("<p>Single Route dijkstra Time: {}ms</p>", t);
      svg << "<path d=\"";
      if (outpoints.size() > 0) {
        auto p = proj->project(outpoints.at(0));
        svg << "M" << p.x << " " << p.y << " ";
      }
      for (auto [s, prev] : std::views::zip(outpoints | std::views::drop(1), outpoints)) {
        auto p = proj->project(s);
        if (fabs(degreefy(s.get_midpoint_latlond()).x - degreefy(prev.get_midpoint_latlond()).x) > 180)
          svg << "M" << p.x << " " << p.y << " ";
        else
          svg << "L" << p.x << " " << p.y << " ";
      }
      svg << R"(" style="stroke: red; fill: none; stroke-width: 1" />)";
    }
    {
      size_t count = 50;
      bool benchmark = qp.has("benchmark");
      form << std::format(
          R"(<tr><td><label for="benchmark">Run Benchmark ({} random dijkstra)</label><td><input type="checkbox" name="benchmark" {}/>)",
          count,
          benchmark ? "checked"sv : ""sv);
      if (benchmark) {
        std::uniform_int_distribution<size_t> distr(0, points.size() - 1);
        dist_type outdist;
        std::vector<sqt> outpoints;
        outpoints.reserve(8000);
        simple_dijkstra<true> djk(points, reverse_points, distances);
        auto start_time = std::chrono::steady_clock::now();
        for (size_t i = 0; i < count; i++) {
          auto start = *(points.begin() + distr(generator));
          auto end = *(points.begin() + distr(generator));
          djk.route(start, reverse_points.at(end), outpoints, outdist);
        }
        double t =
            std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(std::chrono::steady_clock::now() - start_time).count();
        measures << std::format("<p>Random Route dijkstra Time: {}ms for {} ({}ms average)</p>", t, count, t / count);
      }
    }
    if (req->header().path() == "/graph.svg"sv)
      return req->create_response()
          .set_body(std::format(
              R"+(<svg viewBox="0 0 1000 500" style="height:100%; width: auto">{}</svg>)+", svg.str(), form.str(), measures.str()))
          .append_header(restinio::http_field::content_type, "image/svg+xml")
          .done();
    else
      return req->create_response()
          .set_body(std::format(R"+(<html>
  <body style="display: flex; flex-direction: row;">
    <svg viewBox="0 0 1000 500" style="height:100%; width: auto">
      {}
    </svg>
    <form>
      <table style="flex: auto;" border=1>
        {}
        <tr><td><td><button>Refresh</button>
      </table>
      {}
)+",
              svg.str(),
              form.str(),
              measures.str()))
          .append_header(restinio::http_field::content_type, "text/html; charset=utf8")
          .done();
  }));

  return res; // the result from doctest is propagated here as well
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

#if 0
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
