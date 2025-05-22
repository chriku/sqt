#include "osm.hpp"
#include <cmath>
#include <fileformat.pb.h>
#include <filesystem>
#include <forward_list>
#include <fstream>
#include <glm/gtx/string_cast.hpp>
#include <list>
#include <osmformat.pb.h>
#include <print>
#include <ranges>
#include <semaphore>
#include <thread>
#include <zlib.h>

using namespace std::placeholders; // for _1, _2, _3...
using namespace std::literals;

struct osm_reader {
  std::string buffer;
  std::counting_semaphore<2147483647> sema;
  std::filesystem::path path = "/home/christian/sqt/planet-coastlinespbf-cleanedosmpbf.sec";
  std::jthread adder;
  std::list<std::jthread> reader;
  // std::filesystem::path path = "/home/christian/sqt/antarctica-latest.osm.pbf";
  std::mutex node_pairs_mutex;
  std::vector<std::pair<uint64_t, uint64_t>> global_node_pairs;
  std::mutex positions_mutex;
  std::map<uint64_t, sqt> global_positions;
  std::chrono::steady_clock::time_point start1;
  osm_reader()
      : sema(std::jthread::hardware_concurrency() - 1),
        // sema(1),
        adder(std::bind(&osm_reader::run_adder, this, _1)) {
    adder.join();
    std::println("Finished distributing. Waiting for parse threads");
    reader.clear();
    std::println("Parsing OSM File... done in {}s",
        std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::steady_clock::now() - start1).count());
  }
  void run_adder(std::stop_token stop_token) {
    {
      auto start2 = std::chrono::steady_clock::now();
      std::println("Reading OSM File... {}", path.string());
      std::ifstream file(path, std::ios::binary | std::ios::ate);
      file.exceptions(std::ios::badbit);
      std::streamsize size = file.tellg();
      file.seekg(0, std::ios::beg);

      buffer.resize(size);
      if (!file.read(buffer.data(), size)) {
        std::println("Block dead");
        exit(1);
      }
      std::println("Reading OSM File... done in {}s",
          std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::steady_clock::now() - start2).count());
    }
    std::println("Parsing OSM File... with {} threads", std::jthread::hardware_concurrency());
    start1 = std::chrono::steady_clock::now();
    std::string_view buffer_view = buffer;
    size_t i = 0;
    size_t pos = 0;
    OSMPBF::BlobHeader d;
    while (pos < buffer.size()) {
      uint32_t len = 0;
      assert((pos + 4) < buffer.size());
      memcpy(&len, &buffer.at(pos), 4);
      if (std::endian::native != std::endian::big)
        len = std::byteswap(len);
      pos += 4;
      std::string_view hdra = buffer_view.substr(pos, len);
      pos += len;
      if (!d.ParseFromArray(hdra.data(), hdra.size()))
        throw std::runtime_error("Parse Error");
      auto datasize = d.datasize();
      if (i++ == 0)
        run_read<&osm_reader::read_header_block>(std::move(d), pos);
      else
        reader.emplace_back(std::bind(&osm_reader::run_read<&osm_reader::read_primitive_block>, this, std::move(d), pos));
      pos += datasize;

      // if (reader.size() >= 23)
      //  if (reader.size() >= 1)
      // reader.pop_front();
    }
    sema.release(std::jthread::hardware_concurrency() + 1);
  }
  void read_header_block(const std::string& s) {
    OSMPBF::HeaderBlock block;
    if (!block.ParseFromString(s))
      throw std::runtime_error("Parse Error");
    for (auto feature : block.required_features()) {
      if (feature == "OsmSchema-V0.6"s)
        ;
      else if (feature == "DenseNodes"s)
        ;
      else {
        std::println("Missing Required {}", feature);
        exit(1);
      }
    }
  }
  void read_primitive_block(const std::string& s) {
    OSMPBF::PrimitiveBlock block;
    if (!block.ParseFromString(s))
      throw std::runtime_error("Parse Error");
    std::optional<uint32_t> string_natural;
    std::optional<uint32_t> string_coastline;
    for (auto [i, s] : block.stringtable().s() | std::views::enumerate) {
      if (s == "natural"s)
        string_natural = i;
      else if (s == "coastline"s)
        string_coastline = i;
    }
    std::vector<std::pair<uint64_t, uint64_t>> node_pairs;
    std::map<uint64_t, sqt> positions; //(arena);
    for (auto group : block.primitivegroup()) {
      if (string_natural && string_coastline) {
        for (auto way : group.ways()) {
          for (auto [i, k] : way.keys() | std::views::enumerate)
            if (k == *string_natural)
              if (way.vals(i) == *string_coastline) {
                uint64_t id = 0;
                for (auto r : way.refs()) {
                  if (id) {
                    node_pairs.emplace_back(id, id + r);
                  }
                  id += r;
                }
              }
        }
      }
      for (auto node : group.nodes()) {
        std::println("Unimplemented non-dense node: {}", node.id());
        exit(1);
      }
      if (group.has_dense()) {
        double offlat = (.000000001 / 180.0 * M_PI) * block.lat_offset();
        double mullat = (.000000001 / 180.0 * M_PI) * block.granularity();
        double offlon = (.000000001 / 180.0 * M_PI) * block.lon_offset();
        double mullon = (.000000001 / 180.0 * M_PI) * block.granularity();
        double lat = 0;
        double lon = 0;
        uint64_t id = 0;
        for (auto [ido, lato, lono] : std::views::zip(group.dense().id(), group.dense().lat(), group.dense().lon())) {
          id += ido;
          lat += lato;
          lon += lono;
          positions.emplace(id, sqt(glm::dvec3(conv({offlon + (mullon * lon), offlat + (mullat * lat)})), 13));
        }
      }
    }
    {
      std::unique_lock lock(positions_mutex);
      global_positions.merge(std::move(positions));
    }
    {
      std::unique_lock lock(node_pairs_mutex);
      global_node_pairs.insert(global_node_pairs.end(), node_pairs.begin(), node_pairs.end());
    }
  }
  template <auto reader> void run_read(OSMPBF::BlobHeader header, uint64_t file_pos) {
    sema.acquire();
    std::string_view buffer_view = buffer;
    auto dataa = buffer_view.substr(file_pos, header.datasize());
    OSMPBF::Blob blob;
    if (!blob.ParseFromArray(dataa.data(), dataa.size()))
      throw std::runtime_error("Parse Error");
    if (blob.has_raw())
      (this->*reader)(blob.raw());
    else if (blob.has_zlib_data()) {
      std::string s;
      s.resize(blob.raw_size());
      uLongf destlen = s.size();
      if (uncompress(reinterpret_cast<Bytef*>(s.data()),
              &destlen,
              reinterpret_cast<const Bytef*>(blob.zlib_data().data()),
              blob.zlib_data().size()) != Z_OK) {
        std::println("Decompression Error");
        exit(1);
      }
      if (destlen != s.size()) {
        std::println("Decompression Size Error");
        exit(1);
      }
      (this->*reader)(s);
    } else {
      std::println("Invalid Compression Type: {}. Only unserstanding ZLib compressed data", size_t(blob.data_case()));
      exit(1);
    }
    sema.release(1);
  }
};

void read_osm(sqt_tree<tile>& tree) {
  osm_reader r;
  std::println("Marking coasts...");
  static sqt focus = sqt(glm::dvec3(conv({-31.105278 / 180.0 * sqt_impl::PI, 39.6975 / 180.0 * sqt_impl::PI})), 27);
  std::println("FOCUS {}", focus);
  {
    auto start2 = std::chrono::steady_clock::now();
    size_t point_count = 65536;
    size_t thread_count = (r.global_node_pairs.size() / point_count) + 5;
    size_t start = 0;
    std::vector<std::jthread> threads;
    std::mutex mtx;
    std::counting_semaphore<2147483647> sema(std::jthread::hardware_concurrency() * 1.5);
    std::println("STARTING {} threads", thread_count);
    for (size_t i = 0; i < thread_count; i++) {
      threads.emplace_back([&, start, point_count] {
        sema.acquire();
        auto start1 = std::chrono::steady_clock::now();
        std::set<sqt> fin;
        for (auto [curi, bi] : r.global_node_pairs | std::views::drop(start) | std::views::take(point_count)) {
          std::set<sqt>& av = fin;
          {
            auto cur = r.global_positions.at(curi);
            av.insert(cur);
            auto b = r.global_positions.at(bi);
            while (cur != b) {
              sqt nc = std::ranges::min(std::views::concat(std::array<sqt, 1>{cur},
                                            (/*std::views::concat*/ (/*std::array<sqt, 1>{cur},*/ cur.get_neighbors()) |
                                                std::views::transform([](sqt n) { return n.get_neighbors(); })) |
                                                std::views::join),
                  {},
                  [&](sqt n2) { return n2.distance_dvec3(b); });
              if (nc == cur)
                break;
              cur = nc;
              av.insert(cur);
              for (auto n2 : cur.get_neighbors())
                av.insert(n2);
            }
          }
          if constexpr (false) {
            for (size_t i = 0; i < 0; i++) {
              std::set<sqt> next = av;
              for (auto v : av)
                for (auto n : v.get_neighbors())
                  next.insert(n);
              av = next;
            }
            fin.merge(av);
          }
        }
        auto start2 = std::chrono::steady_clock::now();
        std::unique_lock lock(mtx);
        auto start3 = std::chrono::steady_clock::now();
        for (auto v : fin)
          // if (focus.distance_ndvec3(v) < (100.0 / 12000))
          tree.set(v, tile::coast);
        /*std::println("TEXIT {} PATH={}s; QUEUEING={}s; INSERT={}s; POINTS={} from {}",
            start / point_count,
            std::chrono::duration_cast<std::chrono::duration<double>>(start2 - start1).count(),
            std::chrono::duration_cast<std::chrono::duration<double>>(start3 - start2).count(),
            std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::steady_clock::now() - start3).count(),
            fin.size(),
            point_count);*/
        sema.release(1);
      });
      start += point_count;
    }
    threads.clear();
    std::println("Marking coasts... done in {}s",
        std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::steady_clock::now() - start2).count());
  }
}