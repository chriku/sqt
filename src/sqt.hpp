#include "sqt.glsl"
#include <format>
#include <iterator>
#include <ranges>

#include <print>

struct sqt {
  inline sqt() : data_(sqt_impl::sqt_new()) {}
  inline sqt(uint8_t major, std::initializer_list<size_t> minors) : data_(sqt_impl::sqt_new()) {
    sqt_impl::sqt_set_major(data_, major);
    for (auto m : minors) {
      data_ = sqt_impl::sqt_add_minor(data_, m);
    }
  }
  inline sqt(uint8_t major, auto minors) : data_(sqt_impl::sqt_new()) {
    sqt_impl::sqt_set_major(data_, major);
    for (auto m : minors) {
      data_ = sqt_impl::sqt_add_minor(data_, m);
    }
  }
  uint32_t count() const {
    return sqt_impl::sqt_count(data_);
  }
  uint32_t major() const {
    return sqt_impl::sqt_major(data_);
  }
  uint32_t minor(uint32_t i) const {
    return sqt_impl::sqt_minor(data_, i);
  }
  void set_major(uint32_t v) {
    return sqt_impl::sqt_set_major(data_, v);
  }
  sqt add_minor(uint32_t v) const {
    return sqt_impl::sqt_add_minor(data_, v);
  }
  sqt counted(uint32_t v) const {
    sqt d2 = *this;
    sqt_impl::sqt_set_count(d2.data_, v);
    return d2;
  }
  auto operator<=>(const sqt&) const = default;
  std::array<sqt, 3> get_neighbors() const {
    std::array<sqt, 3> ret;
    sqt_impl::sqt_get_neighbors(data_, ret[0].data_, ret[1].data_, ret[2].data_);
    return ret;
  }
  bool is_neighbor(sqt other) const {
    assert(other.count() >= count());
    auto [a, b, c] = other.get_neighbors();
    if (*this == a.counted(count()))
      return true;
    if (*this == b.counted(count()))
      return true;
    if (*this == c.counted(count()))
      return true;
    return false;
  }

  struct iterator {
    using iterator_category = std::random_access_iterator_tag;
    using difference_type = int8_t;
    using value_type = uint8_t;
    sqt_impl::sqt_t data_ = sqt_impl::sqt_new();
    uint8_t index = 0;
    inline value_type operator*() const {
      return sqt_impl::sqt_minor(data_, index);
    }
    inline value_type operator[](difference_type d) const {
      return *((*this) + d);
    }
    inline difference_type operator-(iterator d) const {
      assert(data_ == d.data_);
      return index - d.index;
    }
    inline iterator operator+(difference_type d) const {
      iterator ret = *this;
      assert((long(ret.index) + long(d)) >= 0);
      ret.index += d;
      assert(ret.index <= sqt_impl::sqt_count(data_));
      return ret;
    }
    friend inline iterator operator+(difference_type d, iterator it) {
      return it + d;
    }
    inline iterator operator-(difference_type d) const {
      return (*this) + -d;
    }
    inline iterator& operator+=(difference_type d) {
      *this = (*this) + d;
      return *this;
    }
    inline iterator& operator-=(difference_type d) {
      *this = (*this) - d;
      return *this;
    }
    inline iterator& operator++() {
      index += 1;
      assert(index <= sqt_impl::sqt_count(data_));
      return *this;
    }
    inline iterator& operator--() {
      assert(index > 0);
      index -= 1;
      return *this;
    }
    inline iterator operator++(int) {
      iterator retval = *this;
      ++(*this);
      return retval;
    }
    inline iterator operator--(int) {
      iterator retval = *this;
      --(*this);
      return retval;
    }
    friend constexpr std::partial_ordering operator<=>(iterator a, iterator b) {
      std::partial_ordering v = a.data_ <=> b.data_;
      if ((v == std::partial_ordering::less) || (v == std::partial_ordering::greater))
        return v;
      return a.index <=> b.index;
    }
    constexpr bool operator==(iterator o) const {
      return ((*this) <=> o) == std::strong_ordering::equal;
    }
  };
  inline iterator begin() const {
    return iterator{data_, 0};
  }
  inline iterator end() const {
    return iterator{data_, uint8_t(sqt_impl::sqt_count(data_))};
  }

private:
  inline sqt(sqt_impl::sqt_t v) : data_(v) {}
  sqt_impl::sqt_t data_;
};
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
  inline T* first_leaf(sqt l) const {
    node* n = data.at(l.major());
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
    std::println("A {}/{}", nbr.count(), l.count());
    assert(l.is_neighbor(nbr));
    assert(std::holds_alternative<node_pointer_array>(n->content));
    for (size_t i = 0; i < 4; i++) {
      auto x = nbr.add_minor(i);
      std::println("B {}/{}", x.count(), l.count());
      std::println("ISNBR {} to {}: {}", l, x, l.is_neighbor(x));
      if (l.is_neighbor(x)) {
        const node* n2 = std::get<node_pointer_array>(n->content).at(i);
        std::println("V", n2->content.index());
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

template <> struct std::formatter<sqt> {
  constexpr auto parse(std::format_parse_context& ctx) {
    auto it = ctx.begin();
    if (it == ctx.end())
      return it;
    if (*it != '}')
      throw std::format_error("Invalid format args for sqt.");
    return it;
  }

  auto format(sqt v, std::format_context& ctx) const {
    std::format_to(ctx.out(), "sqt_t({}", v.major());
    for (size_t i = 0; i < v.count(); i++) {
      std::format_to(ctx.out(), ", {}", v.minor(i));
    }
    return std::format_to(ctx.out(), ")");
  }
};
