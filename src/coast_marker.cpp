#include "osm.hpp"
#include <boost/iterator/function_output_iterator.hpp>
#include <list>
#include <print>
#include <vector>

void mark_coast(sqt_tree<tile>& tree) {
  std::list<sqt> todo;
  // auto fl = tree.first_leaf(sqt(0, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));
  auto fl = tree.first_leaf(sqt({0, 0}, 27));
  assert(fl != nullptr);
  if (*fl != tile::undefined) {
    std::println("Invalid zero, zero: {}", size_t(*fl));
    exit(1);
  }
  *fl = tile::water;
  todo.push_back(tree.get_leaf(sqt(0, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0})).value());
  std::vector<sqt> tc;
  tc.reserve(1000);
  while (!todo.empty()) {
    auto v = *todo.begin();
    todo.pop_front();
    auto& d = tree[v];
    assert(d == tile::water);
    tc.clear();
    tree.all_neighbors(v, std::back_inserter(tc));
    for (sqt t : tc) {
      if (tree[t] == tile::undefined) {
        tree[t] = tile::water;
        todo.push_back(t);
      }
    };
  }
}