#ifdef __cplusplus
#include <cstdint>
#include <glm/glm.hpp>
#include <initializer_list>
#define CPP(X) X
#define GLSL(X)
#define INLINE
#define CONSTEXPR constexpr
#define OUT(v, n) v& n
namespace sqt_impl {
  using namespace glm;
#else
#define CPP(X)
#define GLSL(X) X
#define INLINE
#define CONSTEXPR const
#define OUT(v, n) out v n
#define assert(X)
#endif
  struct sqt_t {
    CPP(sqt_t() = delete;)
    CPP(inline sqt_t(std::nullptr_t){};)
    uint64_t _data;
    CPP(auto operator<=>(const sqt_t&) const = default;)
  };
  CONSTEXPR uint64_t major_count = 20;
  CONSTEXPR uint64_t count_bits = 0x1F;
  CONSTEXPR uint64_t major_bits = 0x1F;
  CONSTEXPR uint64_t count_offset = 59;
  CONSTEXPR uint64_t major_offset = 54;
  CONSTEXPR uint64_t count_mask = count_bits << count_offset;
  CONSTEXPR uint64_t major_mask = major_bits << major_offset;
  CONSTEXPR uint64_t inv_count_mask = ~count_mask;
  CONSTEXPR uint64_t inv_major_mask = ~major_mask;

#ifdef __cplusplus
  enum direction_type { direction_A, direction_B, direction_C, direction_center };
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

  INLINE uint32_t sqt_count(sqt_t v) {
    return uint32_t(v._data >> count_offset);
  }
  INLINE uint32_t sqt_major(sqt_t v) {
    return uint32_t((v._data >> major_offset) & major_bits);
  }
  INLINE uint32_t sqt_minor(sqt_t v, uint32_t i) {
    assert(i < sqt_count(v));
    return uint32_t(v._data >> uint64_t(i * 2)) & uint32_t(0x3);
  }
  INLINE void sqt_set_count(OUT(sqt_t, v), uint32_t c) {
    v._data = (v._data & inv_count_mask) | (uint64_t(c) << count_offset);
  }
  INLINE void sqt_set_major(OUT(sqt_t, v), uint32_t m) {
    assert(m < major_count);
    v._data = (v._data & inv_major_mask) | (uint64_t(m) << major_offset);
  }
  INLINE sqt_t sqt_add_minor(sqt_t v, uint32_t m) {
    uint32_t c = sqt_count(v);
    assert((c * 2) <= major_offset);
    sqt_set_count(v, c + 1);
    assert(m < 4);
    v._data = v._data | (uint64_t(m & 0x3) << uint64_t(c * 2));
    return v;
  }
  INLINE sqt_t sqt_new() {
    sqt_t ret CPP((nullptr));
    ret._data = 0;
    return ret;
  }
  INLINE sqt_t sqt_transform_to_adjacent(sqt_t v, uint side) {
    sqt_t ret = sqt_new();
    uint target_primitive = primitive_table[sqt_major(v)][side];
    sqt_set_major(ret, target_primitive);
    for (uint32_t i = 0; i < sqt_count(v); i++) {
      if (sqt_major(v) > 4 && target_primitive > 4 && sqt_major(v) < 15 && target_primitive < 15) {
        if ((side == 2)) {
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
    sqt_set_major(current, sqt_major(v));
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
// INLINE vec3 sqt_get_position(sqt_t v) {}
#ifdef __cplusplus
}
#endif
