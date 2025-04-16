#include "sqt.glsl"
#include <format>
#include <iterator>

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
  auto operator<=>(const sqt&) const = default;
  std::array<sqt, 3> get_neighbors() const {
    std::array<sqt, 3> ret;
    sqt_impl::sqt_get_neighbors(data_, ret[0].data_, ret[1].data_, ret[2].data_);
    return ret;
  }

#if 0
template< class I >
    concept input_or_output_iterator =
        requires(I i) {
            { *i } -> /*can-reference*/;
        } &&
        std::weakly_incrementable<I>;
template< class I >
    concept input_iterator =
        std::input_or_output_iterator<I> &&
        std::indirectly_readable<I> &&
        requires { typename /*ITER_CONCEPT*/<I>; } &&
        std::derived_from</*ITER_CONCEPT*/<I>, std::input_iterator_tag>;
template< class I >
    concept forward_iterator =
        std::input_iterator<I> &&
        std::derived_from</*ITER_CONCEPT*/<I>, std::forward_iterator_tag> &&
        std::incrementable<I> &&
        std::sentinel_for<I, I>;
template< class I >
    concept bidirectional_iterator =
        std::forward_iterator<I> &&
        std::derived_from</*ITER_CONCEPT*/<I>, std::bidirectional_iterator_tag> &&
        requires(I i) {
            { --i } -> std::same_as<I&>;
            { i-- } -> std::same_as<I>;
        };
template< class I >
    concept random_access_iterator =
        std::bidirectional_iterator<I> &&
        std::derived_from</*ITER_CONCEPT*/<I>, std::random_access_iterator_tag> &&
        std::totally_ordered<I> &&
        std::sized_sentinel_for<I, I> &&
        requires(I i, const I j, const std::iter_difference_t<I> n) {
            { i += n } -> std::same_as<I&>;
            { j +  n } -> std::same_as<I>;
            { n +  j } -> std::same_as<I>;
            { i -= n } -> std::same_as<I&>;
            { j -  n } -> std::same_as<I>;
            {  j[n]  } -> std::same_as<std::iter_reference_t<I>>;
        };
#endif
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
