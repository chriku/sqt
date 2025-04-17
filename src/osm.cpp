#include "osm.hpp"
#include <cmath>
#include <fileformat.pb.h>
#include <filesystem>
#include <fstream>
#include <list>
#include <osmformat.pb.h>
#include <print>
#include <ranges>
#include <semaphore>
#include <thread>
#include <zlib.h>

using namespace std::placeholders; // for _1, _2, _3...
using namespace std::literals;

glm::dvec3 conv(glm::dvec2 p);

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
  osm_reader() : sema(std::jthread::hardware_concurrency() - 1), adder(std::bind(&osm_reader::run_adder, this, _1)) {
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
    std::map<uint64_t, sqt> positions;
    for (auto group : block.primitivegroup()) {
      if (string_natural && string_coastline)
        for (auto way : group.ways()) {
          for (auto [i, k] : way.keys() | std::views::enumerate)
            if (k == string_natural.value())
              if (way.vals(i) == string_coastline.value()) {
                // std::println("COAST {}", way.id());
                // if (way.id() == 224779210ULL)
                {
                  std::optional<uint64_t> prev;
                  uint64_t id = 0;
                  for (auto r : way.refs()) {
                    id += r;
                    if (prev) {
                      node_pairs.emplace_back(prev.value(), id);
                    }
                    prev = id;
                  }
                }
              }
        }
      for (auto node : group.nodes()) {
        std::println("Unimplemented non-dense node: {}", node.id());
        exit(1);
      }
      if (group.has_dense()) {
        double lat = 0;
        double lon = 0;
        uint64_t id = 0;
        for (auto [ido, lato, lono] : std::views::zip(group.dense().id(), group.dense().lat(), group.dense().lon())) {
          id += ido;
          lat += lato;
          lon += lono;
          double latitude = .000000001 * (block.lat_offset() + (block.granularity() * lat));
          double longitude = .000000001 * (block.lon_offset() + (block.granularity() * lon));
          positions.emplace(id, sqt(conv(glm::dvec2{longitude / 180.0 * M_PI, latitude / 180.0 * M_PI}), 10));
        }
      }
    }
    {
      std::unique_lock lock(positions_mutex);
      global_positions.merge(positions);
    }
    {
      std::unique_lock lock(node_pairs_mutex);
      for (auto& p : node_pairs)
        global_node_pairs.emplace_back(p);
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
  {
    auto start2 = std::chrono::steady_clock::now();
    size_t thread_count = 32;
    size_t point_count = (r.global_node_pairs.size() / thread_count) + thread_count;
    size_t start = 0;
    std::vector<std::jthread> threads;
    std::mutex mtx;
    for (size_t i = 0; i < thread_count; i++) {
      threads.emplace_back([&, start, point_count] {
        std::set<sqt> fin;
        for (auto [curi, bi] : r.global_node_pairs | std::views::drop(start) | std::views::take(point_count)) {
          std::set<sqt> av;
          {
            auto cur = r.global_positions.at(curi);
            av.insert(cur);
            auto b = r.global_positions.at(bi);
            av.insert(cur);
            av.insert(b);
            while (cur != b) {
              sqt nc = cur;
              for (auto n2 : cur.get_neighbors())
                for (auto n : n2.get_neighbors())
                  if (n.distance_ndvec3(b) < nc.distance_ndvec3(b)) {
                    nc = n;
                  }
              if (nc == cur)
                break;
              cur = nc;
              av.insert(cur);
              for (auto n2 : cur.get_neighbors())
                av.insert(n2);
            }
          }
          {
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
        std::unique_lock lock(mtx);
        for (auto v : fin)
          tree.set(v, tile::coast);
      });
      start += point_count;
    }
    threads.clear();
    std::println("Marking coasts... done in {}s",
        std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::steady_clock::now() - start2).count());
  }
}