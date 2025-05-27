#include "min_heap.hpp"
#include "doctest/doctest.h"

TEST_CASE("simple_heap_action") {
  min_heap<sqt_heap_impl, map_distance_container<sqt>> heap;
  heap.push({sqt(3, {}), 3});
  heap.push({sqt(1, {}), 1});
  heap.push({sqt(2, {}), 2});
  CHECK(!heap.empty());
  CHECK(heap.top().first == sqt(1, {}));
  heap.pop();
  CHECK(!heap.empty());
  CHECK(heap.top().first == sqt(2, {}));
  heap.pop();
  CHECK(!heap.empty());
  CHECK(heap.top().first == sqt(3, {}));
  heap.pop();
  CHECK(heap.empty());
}
TEST_CASE("simple_heap_action") {
  min_heap<sqt_heap_impl, map_distance_container<sqt>> heap;
  heap.push({sqt(3, {}), 5});
  heap.push({sqt(1, {}), 1});
  heap.push({sqt(2, {}), 2});
  CHECK(heap.push({sqt(3, {}), 7}) == false);
  CHECK(heap.push({sqt(3, {}), 3}) == true);
}
TEST_CASE("simple_heap_action") {
  min_heap<idx_heap_impl<3>, fixed_distance_container<3>> heap;
  heap.push({2, 3});
  heap.push({0, 1});
  heap.push({1, 2});
  CHECK(!heap.empty());
  CHECK(heap.top().first == 0);
  heap.pop();
  CHECK(!heap.empty());
  CHECK(heap.top().first == 1);
  heap.pop();
  CHECK(!heap.empty());
  CHECK(heap.top().first == 2);
  heap.pop();
  CHECK(heap.empty());
}
TEST_CASE("simple_heap_action") {
  min_heap<idx_heap_impl<3>, fixed_distance_container<3>> heap;
  heap.push({2, 5});
  heap.push({0, 1});
  heap.push({1, 2});
  CHECK(heap.push({2, 7}) == false);
  CHECK(heap.push({2, 3}) == true);
}