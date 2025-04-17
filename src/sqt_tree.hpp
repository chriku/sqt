#ifndef SQT_TREE_HPP
#define SQT_TREE_HPP

#include "sqt.hpp"

template <typename T, typename Allocator = std::allocator<T>> struct sqt_tree {
  inline sqt_tree(const T& t = {}) {
    for (auto& n : data)
      n.content = t;
  }
  inline void set(sqt l, T&& v) {
    node* n = breakdown(l);
    if (!std::holds_alternative<T>(n->content)) {
      delete_npa(std::get<node_pointer_array>(n->content));
    }
    n->content = std::move(v);
  }
  inline bool touches_leaf(sqt l) const {
    const node* n = &data.at(l.major());
    for (auto step : l) {
      if (std::holds_alternative<T>(n->content))
        return true;
      n = std::get<node_pointer_array>(n->content).at(step);
    }
    return std::holds_alternative<T>(n->content);
  }
  inline std::optional<sqt> get_leaf(sqt l) const {
    const node* n = &data.at(l.major());
    for (auto [i, step] : l | std::views::enumerate) {
      if (std::holds_alternative<T>(n->content))
        return l.counted(i);
      n = std::get<node_pointer_array>(n->content).at(step);
    }
    if (std::holds_alternative<T>(n->content))
      return l;
    return {};
  }
  inline bool is_leaf(sqt l) const {
    return std::holds_alternative<T>(lookdown(l)->content);
  }
  inline T* first_leaf(sqt l) {
    node* n = &data.at(l.major());
    for (auto step : l) {
      if (std::holds_alternative<T>(n->content))
        return &std::get<T>(n->content);
      n = std::get<node_pointer_array>(n->content).at(step);
    }
    if (std::holds_alternative<T>(n->content))
      return &std::get<T>(n->content);
    return nullptr;
  }
  inline T& operator[](sqt l) {
    return std::get<T>(lookdown(l)->content);
  }
  inline void all_neighbors(sqt l, std::output_iterator<sqt> auto outputter) const {
    assert(is_leaf(l));
    for (auto n : l.get_neighbors()) {
      auto leaf = get_leaf(n);
      if (leaf)
        *outputter++ = leaf.value();
      else
        find_small_neighbors(l, n, lookdown(n), outputter);
    }
  }

private:
  struct node;
  using node_pointer_array = std::array<node*, 4>;
  inline void find_small_neighbors(sqt l, sqt nbr, const node* n, std::output_iterator<sqt> auto& outputter) const {
    assert(l.is_neighbor(nbr));
    assert(std::holds_alternative<node_pointer_array>(n->content));
    for (size_t i = 0; i < 4; i++) {
      auto x = nbr.add_minor(i);
      if (l.is_neighbor(x)) {
        const node* n2 = std::get<node_pointer_array>(n->content).at(i);
        if (std::holds_alternative<T>(n2->content)) {
          *outputter++ = x;
        } else {
          find_small_neighbors(l, x, n2, outputter);
        }
      }
    }
  }
  inline node* breakdown(sqt l) {
    node* n = &data.at(l.major());
    for (auto step : l) {
      if (std::holds_alternative<T>(n->content)) {
        node_pointer_array val;
        for (auto& v : val) {
          v = new node;
          v->content = std::get<T>(n->content);
        }
        n->content = val;
      }
      n = std::get<node_pointer_array>(n->content).at(step);
    }
    return n;
  }
  inline const node* lookdown(sqt l) const {
    const node* n = &data.at(l.major());
    for (auto step : l) {
      assert(std::holds_alternative<node_pointer_array>(n->content));
      n = std::get<node_pointer_array>(n->content).at(step);
    }
    return n;
  }
  inline node* lookdown(sqt l) {
    node* n = &data.at(l.major());
    for (auto step : l) {
      assert(std::holds_alternative<node_pointer_array>(n->content));
      n = std::get<node_pointer_array>(n->content).at(step);
    }
    return n;
  }
  inline void delete_npa(node_pointer_array& a) {
    for (auto n : a) {
      if (std::holds_alternative<node_pointer_array>(n->content))
        delete_npa(std::get<node_pointer_array>(n->content));
      delete n;
    }
  }
  struct node {
    std::variant<node_pointer_array, T> content;
  };
  std::array<node, 20> data;
};

#endif // SQT_TREE_HPP
