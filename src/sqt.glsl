#ifdef __cplusplus
#include <array>
#include <cstdint>
#include <glm/glm.hpp>
#include <initializer_list>
#define CPP(X) X
#define GLSL(X)
#define CONST_INLINE [[gnu::const]] inline
#define INLINE inline
#define CONSTEXPR constexpr
#define OUT(v, n) v& n
#define assume(X) [[assume(X)]]
// #define assume(X) assert(X)
#define retarray(T, n) std::array<T, n>
namespace sqt_impl {
  using namespace glm;
#else
#define CPP(X)
#define GLSL(X) X
#define CONST_INLINE
#define INLINE
#define CONSTEXPR const
#define OUT(v, n) out v n
#define assert(X)
#define assume(X)
#define retarray(T, n) T[n]
#endif
  struct sqt_t {
    CPP(sqt_t() = delete;)
    CPP(inline sqt_t(std::nullptr_t){};)
    uint64_t _data;
    CPP(inline auto operator<=>(const sqt_t&) const = default;)
  };
  CONSTEXPR uint64_t minor_count = 27;
  CONSTEXPR uint64_t major_count = 20;
  CONSTEXPR uint64_t count_bits = 0x1F;
  CONSTEXPR uint64_t major_bits = 0x1F;
  CONSTEXPR uint64_t count_offset = 59;
  CONSTEXPR uint64_t major_offset = 54;
  CONSTEXPR uint64_t count_mask = count_bits << count_offset;
  CONSTEXPR uint64_t major_mask = major_bits << major_offset;
  CONSTEXPR uint64_t minor_mask = (uint64_t(1) << major_offset) - 1;
  CONSTEXPR uint64_t inv_count_mask = ~count_mask;
  CONSTEXPR uint64_t inv_major_mask = ~major_mask;
  CONSTEXPR uint64_t inv_minor_mask = ~minor_mask;

#ifdef __cplusplus
  enum direction_type : uint8_t { direction_A, direction_B, direction_C, direction_center };
#else
CONSTEXPR uint8_t direction_A = uint8_t(0);
CONSTEXPR uint8_t direction_B = uint8_t(1);
CONSTEXPR uint8_t direction_C = uint8_t(2);
CONSTEXPR uint8_t direction_center = uint8_t(3);
#define direction_type uint8_t
#endif

  CONSTEXPR uint8_t permutations[6][3] = {{uint8_t(0), uint8_t(1), uint8_t(2)},
      {uint8_t(0), uint8_t(2), uint8_t(1)},
      {uint8_t(1), uint8_t(2), uint8_t(0)},
      {uint8_t(1), uint8_t(0), uint8_t(2)},
      {uint8_t(2), uint8_t(0), uint8_t(1)},
      {uint8_t(2), uint8_t(1), uint8_t(0)}};
  CONSTEXPR uint8_t state_table[6][4] = {{uint8_t(1), uint8_t(5), uint8_t(3), uint8_t(0)},
      {uint8_t(0), uint8_t(2), uint8_t(4), uint8_t(1)},
      {uint8_t(3), uint8_t(1), uint8_t(5), uint8_t(2)},
      {uint8_t(2), uint8_t(4), uint8_t(0), uint8_t(3)},
      {uint8_t(5), uint8_t(3), uint8_t(1), uint8_t(4)},
      {uint8_t(4), uint8_t(0), uint8_t(2), uint8_t(5)}};
  CONSTEXPR uint8_t primitive_table[20][3] = {{uint8_t(5), uint8_t(4), uint8_t(1)},
      {uint8_t(6), uint8_t(0), uint8_t(2)},
      {uint8_t(7), uint8_t(1), uint8_t(3)},
      {uint8_t(8), uint8_t(2), uint8_t(4)},
      {uint8_t(9), uint8_t(3), uint8_t(0)},
      {uint8_t(0), uint8_t(10), uint8_t(14)},
      {uint8_t(1), uint8_t(11), uint8_t(10)},
      {uint8_t(2), uint8_t(12), uint8_t(11)},
      {uint8_t(3), uint8_t(13), uint8_t(12)},
      {uint8_t(4), uint8_t(14), uint8_t(13)},
      {uint8_t(15), uint8_t(5), uint8_t(6)},
      {uint8_t(16), uint8_t(6), uint8_t(7)},
      {uint8_t(17), uint8_t(7), uint8_t(8)},
      {uint8_t(18), uint8_t(8), uint8_t(9)},
      {uint8_t(19), uint8_t(9), uint8_t(5)},
      {uint8_t(10), uint8_t(16), uint8_t(19)},
      {uint8_t(11), uint8_t(17), uint8_t(15)},
      {uint8_t(12), uint8_t(18), uint8_t(16)},
      {uint8_t(13), uint8_t(19), uint8_t(17)},
      {uint8_t(14), uint8_t(15), uint8_t(18)}};
  CONSTEXPR double PI = 3.14159265358979323846;
  CONSTEXPR double DEG36 = 0.62831853071795864768;
  CONSTEXPR double DEG72 = 1.25663706143591729537;
  CONSTEXPR double DEG90 = PI / 2;
  CONSTEXPR double DEG108 = 1.88495559215387594306;
  CONSTEXPR double DEG120 = 2.09439510239319549229;
  CONSTEXPR double DEG144 = 2.51327412287183459075;
  CONSTEXPR double DEG180 = PI;

  /* 26.565051177 degrees */
  CONSTEXPR double V_LAT = 0.46364760899944494524;

  /* 52.62263186 */
  CONSTEXPR double E_RAD = 0.91843818702186776133;

  /* 10.81231696 */
  CONSTEXPR double F_RAD = 0.18871053072122403508;

  CONSTEXPR dvec2 vertex[12] = {dvec2(0.0, DEG90),
      dvec2(DEG180, V_LAT),
      dvec2(-DEG108, V_LAT),
      dvec2(-DEG36, V_LAT),
      dvec2(DEG36, V_LAT),
      dvec2(DEG108, V_LAT),
      dvec2(-DEG144, -V_LAT),
      dvec2(-DEG72, -V_LAT),
      dvec2(0.0, -V_LAT),
      dvec2(DEG72, -V_LAT),
      dvec2(DEG144, -V_LAT),
      dvec2(0.0, -DEG90)};
  CONSTEXPR uint8_t index_table[20][3] = {{uint8_t(0), uint8_t(1), uint8_t(2)},
      {uint8_t(0), uint8_t(2), uint8_t(3)},
      {uint8_t(0), uint8_t(3), uint8_t(4)},
      {uint8_t(0), uint8_t(4), uint8_t(5)},
      {uint8_t(0), uint8_t(5), uint8_t(1)},
      {uint8_t(6), uint8_t(2), uint8_t(1)},
      {uint8_t(7), uint8_t(3), uint8_t(2)},
      {uint8_t(8), uint8_t(4), uint8_t(3)},
      {uint8_t(9), uint8_t(5), uint8_t(4)},
      {uint8_t(10), uint8_t(1), uint8_t(5)},
      {uint8_t(2), uint8_t(6), uint8_t(7)},
      {uint8_t(3), uint8_t(7), uint8_t(8)},
      {uint8_t(4), uint8_t(8), uint8_t(9)},
      {uint8_t(5), uint8_t(9), uint8_t(10)},
      {uint8_t(1), uint8_t(10), uint8_t(6)},
      {uint8_t(11), uint8_t(7), uint8_t(6)},
      {uint8_t(11), uint8_t(8), uint8_t(7)},
      {uint8_t(11), uint8_t(9), uint8_t(8)},
      {uint8_t(11), uint8_t(10), uint8_t(9)},
      {uint8_t(11), uint8_t(6), uint8_t(10)}};

  CONST_INLINE uint32_t sqt_count(sqt_t v) {
    uint32_t ret = uint32_t(v._data >> count_offset);
    assume(ret <= 27);
    return ret;
  }
  CONST_INLINE uint32_t sqt_major(sqt_t v) {
    uint32_t ret = uint32_t((v._data >> major_offset) & major_bits);
    assume(ret < 20);
    return ret;
  }
  CONST_INLINE uint32_t sqt_minor(sqt_t v, uint32_t i) {
    assert(i < sqt_count(v));
    assume(i < sqt_count(v));
    assume(i < minor_count);
    uint32_t ret = uint32_t(v._data >> uint64_t(i * 2)) & uint32_t(0x3);
    assume(ret < 20);
    return ret;
  }
  CONST_INLINE sqt_t sqt_set_count(sqt_t v, uint32_t c) {
    assert(c <= minor_count);
    assume(c <= minor_count);
    v._data = (v._data & inv_count_mask & (inv_minor_mask | ((uint64_t(1) << (c * 2)) - 1))) | (uint64_t(c) << count_offset);
    return v;
  }
  CONST_INLINE sqt_t sqt_set_major(sqt_t v, uint32_t m) {
    assert(m < major_count);
    assume(m < major_count);
    v._data = (v._data & inv_major_mask) | (uint64_t(m) << major_offset);
    return v;
  }
  CONST_INLINE sqt_t sqt_add_minor(sqt_t v, uint32_t m) {
    uint32_t c = sqt_count(v);
    assert(c <= minor_count);
    assume(c <= minor_count);
    v = sqt_set_count(v, c + 1);
    assert(m < 4);
    assume(m < 4);
    v._data = v._data | (uint64_t(m & 0x3) << uint64_t(c * 2));
    return v;
  }
  CONST_INLINE sqt_t sqt_new() {
    sqt_t ret CPP((nullptr));
    ret._data = 0;
    return ret;
  }
  CONST_INLINE sqt_t sqt_transform_to_adjacent(sqt_t v, uint side) {
    sqt_t ret = sqt_new();
    uint target_primitive = primitive_table[sqt_major(v)][side];
    assume(target_primitive < 20);
    ret = sqt_set_major(ret, target_primitive);
    for (uint32_t i = 0; i < sqt_count(v); i++) {
      if (sqt_major(v) > 4 && target_primitive > 4 && sqt_major(v) < 15 && target_primitive < 15) {
        if (side == 2) {
          switch (sqt_minor(v, i)) {
          case 0:
            ret = sqt_add_minor(ret, 1);
            break;
          case 1:
            ret = sqt_add_minor(ret, 0);
            break;
          case 2:
            ret = sqt_add_minor(ret, 2);
            break;
          default:
            assert(!"Invalid field array value");
            break;
          }
        } else {
          switch (sqt_minor(v, i)) {
          case 0:
            ret = sqt_add_minor(ret, 2);
            break;
          case 1:
            ret = sqt_add_minor(ret, 1);
            break;
          case 2:
            ret = sqt_add_minor(ret, 0);
            break;
          default:
            assert(!"Invalid field array value");
            break;
          }
        }
      } else {
        switch (sqt_minor(v, i)) {
        case 0:
          ret = sqt_add_minor(ret, 0);
          break;
        case 1:
          ret = sqt_add_minor(ret, 2);
          break;
        case 2:
          ret = sqt_add_minor(ret, 1);
          break;
        default:
          assert(!"Invalid field array value");
          break;
        }
      }
    }
    return ret;
  }
  INLINE void sqt_get_neighbors(sqt_t v, OUT(sqt_t, n0), OUT(sqt_t, n1), OUT(sqt_t, n2)) {
    sqt_t n[3] = {sqt_new(), sqt_new(), sqt_new()};
    bool is_good[3] = {false, false, false};
    sqt_t current = sqt_new();
    current = sqt_set_major(current, sqt_major(v));
    uint32_t current_state = 0;
    for (uint32_t i = 0; i < sqt_count(v); i++) {
      uint32_t next = sqt_minor(v, i);
      uint32_t per0 = permutations[current_state][0];
      uint32_t per1 = permutations[current_state][1];
      uint32_t per2 = permutations[current_state][2];
      switch (next) {
      case direction_A:
        n[per0] = sqt_add_minor(current, direction_center);
        is_good[per0] = true;
        n[per1] = sqt_add_minor(n[per1], direction_A);
        n[per2] = sqt_add_minor(n[per2], direction_A);
        break;
      case 1:
        n[per0] = sqt_add_minor(n[per0], direction_B);
        n[per1] = sqt_add_minor(current, direction_center);
        is_good[per1] = true;
        n[per2] = sqt_add_minor(n[per2], direction_B);
        break;
      case 2:
        n[per0] = sqt_add_minor(n[per0], direction_C);
        n[per1] = sqt_add_minor(n[per1], direction_C);
        n[per2] = sqt_add_minor(current, direction_center);
        is_good[per2] = true;
        break;
      case 3:
        n[per0] = sqt_add_minor(current, direction_A);
        n[per1] = sqt_add_minor(current, direction_B);
        n[per2] = sqt_add_minor(current, direction_C);
        is_good[per0] = true;
        is_good[per1] = true;
        is_good[per2] = true;
        break;
      default:
        assert(false);
        break;
      }
      current = sqt_add_minor(current, next);
      current_state = state_table[current_state][next];
    }
    if (!is_good[0])
      n[0] = sqt_transform_to_adjacent(v, 0);
    if (!is_good[1])
      n[1] = sqt_transform_to_adjacent(v, 1);
    if (!is_good[2])
      n[2] = sqt_transform_to_adjacent(v, 2);
    n0 = n[0];
    n1 = n[1];
    n2 = n[2];
  }

  CONST_INLINE vec3 sqt_get_point_vec3(uint i) {
    vec2 p = vec2(vertex[i] + dvec2(PI, PI / 2));
    return vec3(sin(p.y) * cos(p.x), sin(p.y) * sin(p.x), cos(p.y));
  }
  CONST_INLINE vec3 sqt_cut_edge_vec3(vec3 a, vec3 b) {
    return (a + b) / 2.0f;
  }
  CONST_INLINE vec3 sqt_calc_midpoint_vec3(vec3 a, vec3 b, vec3 c) {
    return (a + b + c) / 3.0f;
  }

  CONST_INLINE vec3 sqt_get_point_nvec3(uint i) {
    vec2 p = vec2(vertex[i] + dvec2(PI, PI / 2));
    return vec3(sin(p.y) * cos(p.x), sin(p.y) * sin(p.x), cos(p.y));
  }
  CONST_INLINE vec3 sqt_cut_edge_nvec3(vec3 a, vec3 b) {
    return normalize((a + b) / 2.0f);
  }
  CONST_INLINE vec3 sqt_calc_midpoint_nvec3(vec3 a, vec3 b, vec3 c) {
    return normalize((a + b + c) / 3.0f);
  }

  CONST_INLINE dvec3 sqt_get_point_dvec3(uint i) {
    dvec2 p = dvec2(vertex[i] + dvec2(PI, PI / 2));
    return dvec3(sin(float(p.y)) * cos(float(p.x)), sin(float(p.y)) * sin(float(p.x)), cos(float(p.y)));
  }
  CONST_INLINE dvec3 sqt_cut_edge_dvec3(dvec3 a, dvec3 b) {
    return (a + b) / 2.0;
  }
  CONST_INLINE dvec3 sqt_calc_midpoint_dvec3(dvec3 a, dvec3 b, dvec3 c) {
    return (a + b + c) / 3.0;
  }

  CONST_INLINE dvec3 sqt_get_point_ndvec3(uint i) {
    dvec2 p = dvec2(vertex[i] + dvec2(PI, PI / 2));
    return dvec3(sin(float(p.y)) * cos(float(p.x)), sin(float(p.y)) * sin(float(p.x)), cos(float(p.y)));
  }
  CONST_INLINE dvec3 sqt_cut_edge_ndvec3(dvec3 a, dvec3 b) {
    return normalize((a + b) / 2.0);
  }
  CONST_INLINE dvec3 sqt_calc_midpoint_ndvec3(dvec3 a, dvec3 b, dvec3 c) {
    return normalize((a + b + c) / 3.0);
  }
  CONST_INLINE double sqt_distance_ndvec3(dvec3 a, dvec3 b) {
    return length(a - b);
  }

#define IMPL_IO(name, T)                                                                                                                   \
  CONST_INLINE retarray(T, 3) sqt_get_point_subdiv_##name(uint minor, T oa, T ob, T oc) {                                                  \
    retarray(T, 3) ret = {oa, ob, oc};                                                                                                     \
    assert(minor < 4);                                                                                                                     \
    if (minor != 0)                                                                                                                        \
      ret[0] = sqt_cut_edge_##name(ob, oc);                                                                                                \
    if (minor != 1)                                                                                                                        \
      ret[2] = sqt_cut_edge_##name(oa, ob);                                                                                                \
    if (minor != 2)                                                                                                                        \
      ret[1] = sqt_cut_edge_##name(oa, oc);                                                                                                \
    return ret;                                                                                                                            \
  }                                                                                                                                        \
  CONST_INLINE retarray(T, 3) sqt_get_points_##name(sqt_t v) {                                                                             \
    uint major = sqt_major(v);                                                                                                             \
    assume(major < 20);                                                                                                                    \
    T a = sqt_get_point_##name(index_table[major][0]);                                                                                     \
    T b = sqt_get_point_##name(index_table[major][1]);                                                                                     \
    T c = sqt_get_point_##name(index_table[major][2]);                                                                                     \
    const uint count = sqt_count(v);                                                                                                       \
    assume(count < minor_count);                                                                                                           \
    for (uint i = 0; i < count; i++) {                                                                                                     \
      uint minor = sqt_minor(v, i);                                                                                                        \
      retarray(T, 3) subdiv = sqt_get_point_subdiv_##name(minor, a, b, c);                                                                 \
      a = subdiv[0];                                                                                                                       \
      b = subdiv[1];                                                                                                                       \
      c = subdiv[2];                                                                                                                       \
    }                                                                                                                                      \
    retarray(T, 3) ret = {a, b, c};                                                                                                        \
    return ret;                                                                                                                            \
  }                                                                                                                                        \
  CONST_INLINE T sqt_get_midpoint_##name(sqt_t v) {                                                                                        \
    retarray(T, 3) vals = sqt_get_points_##name(v);                                                                                        \
    return sqt_calc_midpoint_##name(vals[0], vals[1], vals[2]);                                                                            \
  }                                                                                                                                        \
  CONST_INLINE T sqt_get_major_midpoint_##name(uint major) {                                                                               \
    return sqt_calc_midpoint_##name(sqt_get_point_##name(index_table[major][0]),                                                           \
        sqt_get_point_##name(index_table[major][1]),                                                                                       \
        sqt_get_point_##name(index_table[major][2]));                                                                                      \
  }

  IMPL_IO(vec3, vec3)
  IMPL_IO(nvec3, vec3)
  IMPL_IO(dvec3, dvec3)
  IMPL_IO(ndvec3, dvec3)

  CONST_INLINE sqt_t sqt_from_point_ndvec3(dvec3 pos, uint granularity) {
    assert(granularity <= minor_count);
    assume(granularity <= minor_count);
    sqt_t v = sqt_new();
    {
      double distance = 1000;
      uint major = 0;
      for (uint i = 0; i < 20; i++) {
        double d = sqt_distance_ndvec3(pos, sqt_get_major_midpoint_ndvec3(i));
        assume(d < 10);
        if (d < distance) {
          major = i;
          distance = d;
        }
      }
      v = sqt_set_major(v, major);
    }
    for (uint i = 0; i < granularity; i++) {
      retarray(dvec3, 3) p = sqt_get_points_ndvec3(v);
      double distance = 1000;
      uint minor = 0;
      for (uint i = 0; i < 4; i++) {
        retarray(dvec3, 3) sd = sqt_get_point_subdiv_ndvec3(i, p[0], p[1], p[2]);
        double d = sqt_distance_ndvec3(pos, sqt_calc_midpoint_ndvec3(sd[0], sd[1], sd[2]));
        assume(d < 10);
        if (d < distance) {
          minor = i;
          distance = d;
        }
      }
      v = sqt_add_minor(v, minor);
    }
    return v;
  }

// INLINE vec3 sqt_get_position(sqt_t v) {}
#ifdef __cplusplus
}
#endif
