#include "osm.hpp"
#include <list>
#include <print>
#include <vector>

void mark_coast(sqt_tree<tile>& tree) {
  std::list<sqt> todo;
  // auto fl = tree.first_leaf(sqt(0, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));
  for (auto root : {sqt({0, 0}, 27), sqt({34.25509008417936 / 180.0 * sqt_impl::PI, 43.291032400186936 / 180.0 * sqt_impl::PI}, 27)}) {
    auto fl = tree.first_leaf(root);
    assert(fl != nullptr);
    if (*fl != tile::undefined) {
      std::println("Invalid zero, zero: {}", size_t(*fl));
      exit(1);
    }
    *fl = tile::water;
    todo.push_back(tree.get_leaf(root).value());
  }
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