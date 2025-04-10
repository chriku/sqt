#ifdef __cplusplus
#include <glm/glm.hpp>
#include <cstdint>
#define CPP(X) X
#define GLSL(X) 
#define INLINE 
#define CONSTEXPR constexpr
using uint = uint32_t;
namespace sqt {
using namespace glm;
#else
#define CPP(X) 
#define GLSL(X) X
#define INLINE 
#define CONSTEXPR const
#endif
struct sqt_t {
uint64_t _data;
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
INLINE uint sqt_count(sqt_t v) {
  return v._data >> count_offset;
}
INLINE uint sqt_major(sqt_t v) {
  return (v._data >> major_offset) & major_bits;
}
INLINE uint sqt_set_count(sqt_t v, uint c) {
  return (v._data & inv_major_mask) | (c << count_bits);
}
INLINE uint sqt_set_major(sqt_t v, uint m) {
  assert(v < major_count);
  return (v._data & inv_major_mask) | (m << count_bits);
}
#ifdef __cplusplus
}
#endif
