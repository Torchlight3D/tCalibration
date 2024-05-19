#pragma once

#include <cassert>
#include <map>
#include <optional>
#include <stdexcept>
#include <unordered_map>

#define REQUIRES_THAT(TYPENAME, EXPRESSION) \
    typename _T_ = TYPENAME, typename = std::void_t<decltype(EXPRESSION)>

namespace tl::impl {

namespace traits {
template <typename T, typename = std::void_t<>>
struct is_bidirectional
{
    static constexpr bool value = false;
};

template <typename T>
struct is_bidirectional<
    T, std::void_t<typename std::iterator_traits<T>::iterator_category>>
{
    static constexpr bool value =
        std::is_base_of_v<std::bidirectional_iterator_tag,
                          typename std::iterator_traits<T>::iterator_category>;
};

template <typename T>
constexpr inline bool is_bidirectional_v = is_bidirectional<T>::value;

template <typename T>
struct is_multimap
{
    static constexpr bool value = false;
};

template <typename Key, typename Val, typename Comp, typename Alloc>
struct is_multimap<std::multimap<Key, Val, Comp, Alloc>>
{
    static constexpr bool value = true;
};

template <typename Key, typename Val, typename Hash, typename Comp,
          typename Alloc>
struct is_multimap<std::unordered_multimap<Key, Val, Hash, Comp, Alloc>>
{
    static constexpr bool value = true;
};

template <typename T>
constexpr inline bool is_multimap_v = is_multimap<T>::value;

template <typename T>
constexpr inline bool nothrow_comparable =
    noexcept(std::declval<T>() == std::declval<T>());
} // namespace traits

template <typename T>
struct get_first_impl
{
    template <typename U>
    constexpr auto &&operator()(U &&value) noexcept
    {
        return std::forward<U>(value);
    }
};

template <typename T, typename U>
struct get_first_impl<std::pair<T, U>>
{
    template <typename Pair>
    constexpr auto &&operator()(Pair &&pair) noexcept
    {
        return std::forward<Pair>(pair).first;
    }
};

template <typename T>
constexpr auto &&get_first(T &&val) noexcept
{
    return get_first_impl<std::remove_const_t<std::remove_reference_t<T>>>{}(
        std::forward<T>(val));
}

template <typename T>
class AllocOncePointer
{
public:
    constexpr AllocOncePointer(T *data = nullptr) noexcept
        : data(data), owner(false)
    {
    }

    template <typename... Args>
    explicit AllocOncePointer(Args &&...args)
        : data(new T(std::forward<Args>(args)...)), owner(true)
    {
    }

    constexpr AllocOncePointer(const AllocOncePointer &other) noexcept
        : data(other.data), owner(false)
    {
    }

    constexpr void swap(AllocOncePointer &other) noexcept
    {
        std::swap(data, other.data);
        std::swap(owner, other.owner);
    }

    constexpr AllocOncePointer(AllocOncePointer &&other) noexcept
        : AllocOncePointer()
    {
        swap(other);
    }

    constexpr AllocOncePointer &operator=(AllocOncePointer other) noexcept
    {
        swap(other);
        return *this;
    }

    ~AllocOncePointer()
    {
        if (owner) {
            delete data;
        }
    }

    [[nodiscard]] constexpr bool isOwner() const noexcept { return owner; }

    constexpr T &operator*() noexcept { return *data; }
    constexpr const T &operator*() const noexcept { return *data; }

    constexpr T *operator->() noexcept { return data; }
    constexpr const T *operator->() const noexcept { return data; }

    constexpr bool operator==(const AllocOncePointer &other) const noexcept
    {
        return data == other.data;
    }
    friend constexpr bool operator==(const AllocOncePointer &lhs,
                                     std::nullptr_t) noexcept
    {
        return lhs.data == nullptr;
    }
    friend constexpr bool operator==(std::nullptr_t,
                                     const AllocOncePointer &rhs) noexcept
    {
        return rhs.data == nullptr;
    }

    friend constexpr bool operator!=(const AllocOncePointer &lhs,
                                     std::nullptr_t) noexcept
    {
        return lhs.data != nullptr;
    }
    friend constexpr bool operator!=(std::nullptr_t,
                                     const AllocOncePointer &rhs) noexcept
    {
        return rhs.data != nullptr;
    }

    constexpr bool operator!=(const T *other) const noexcept
    {
        return !(*this == other);
    }

private:
    T *data;
    bool owner{};
};

template <typename T>
constexpr void swap(AllocOncePointer<T> &a, AllocOncePointer<T> &b) noexcept
{
    a.swap(b);
}

template <typename T>
class Surrogate
{
public:
    constexpr Surrogate(T *data) noexcept : data(data) {}

    constexpr bool operator==(Surrogate other) const
        noexcept(traits::nothrow_comparable<T>)
    {
        return *data == *other.data;
    }

    constexpr bool operator!=(Surrogate other) const
        noexcept(noexcept(other == other))
    {
        return !(*this == other);
    }

    constexpr T &operator*() noexcept { return *data; }
    constexpr const T &operator*() const noexcept { return *data; }

    constexpr T *operator->() noexcept { return data; }
    constexpr const T *operator->() const noexcept { return data; }

    constexpr T *get() noexcept { return data; }
    constexpr const T *get() const noexcept { return data; }

private:
    T *data;
};

} // namespace tl::impl

namespace tl {

template <typename ForwardKey, typename InverseKey,
          template <typename...> typename ForwardMapType = std::unordered_map,
          template <typename...> typename InverseMapType = std::unordered_map>
class bimap
{
    using ForwardMap =
        ForwardMapType<ForwardKey, impl::Surrogate<const InverseKey>>;
    using InverseBiMap =
        bimap<InverseKey, ForwardKey, InverseMapType, ForwardMapType>;
    friend InverseBiMap;
    using InverseMap = typename InverseBiMap::ForwardMap;
    using InversBiMapPtr = impl::AllocOncePointer<InverseBiMap>;

    friend class impl::AllocOncePointer<bimap>;

    static_assert(std::is_default_constructible_v<ForwardMap>,
                  "ForwardMap base containers must be default constructable.");
    static_assert(std::is_copy_constructible_v<ForwardMap>,
                  "ForwardMap base containers must be copy constructable.");

    explicit bimap(InverseBiMap &inverseMap) noexcept(
        std::is_nothrow_default_constructible_v<ForwardMap>)
        : map(), inverseAccess(&inverseMap)
    {
    }

public:
    class iterator
    {
        using IteratorType =
            decltype(std::declval<std::add_const_t<ForwardMap>>().begin());
        static constexpr bool copy_constructable =
            std::is_nothrow_copy_constructible_v<IteratorType>;
        static constexpr bool copy_assignable =
            std::is_nothrow_copy_assignable_v<IteratorType>;

        friend class bimap;
        friend InverseBiMap;

    public:
        using value_type = std::pair<const ForwardKey &, const InverseKey &>;
        using reference = value_type;
        using pointer = std::add_pointer_t<std::add_const_t<value_type>>;
        using diference_type =
            typename std::iterator_traits<IteratorType>::difference_type;
        using iterator_category =
            typename std::iterator_traits<IteratorType>::iterator_category;

        constexpr explicit iterator(const IteratorType &it) noexcept(
            copy_constructable)
            : it(it)
        {
        }

        constexpr iterator(const iterator &other) noexcept(
            std::is_constructible_v<iterator, IteratorType>)
            : iterator(other.it)
        {
        }

        constexpr iterator(iterator &&other) noexcept(
            std::is_constructible_v<iterator, IteratorType>)
            : iterator(other)
        {
        }

        constexpr iterator &operator=(iterator other) noexcept(copy_assignable)
        {
            it = other.it;
            return *this;
        }

        ~iterator() = default;

        constexpr iterator &operator++() noexcept(
            noexcept(++std::declval<IteratorType>()))
        {
            ++it;
            return *this;
        }

        constexpr iterator operator++(int) noexcept(
            std::is_nothrow_copy_constructible_v<iterator> &&
            noexcept(++std::declval<iterator>()))
        {
            auto tmp = *this;
            ++*this;
            return tmp;
        }

        template <bool IsBidirectional =
                      impl::traits::is_bidirectional_v<IteratorType>>
        constexpr auto operator--() noexcept(
            noexcept(--std::declval<IteratorType>()))
            -> std::enable_if_t<IsBidirectional, iterator &>
        {
            --it;
            return *this;
        }

        template <bool IsBidirectional =
                      impl::traits::is_bidirectional_v<IteratorType>>
        constexpr auto operator--(int) noexcept(
            std::is_nothrow_copy_constructible_v<iterator> &&
            noexcept(--std::declval<iterator>()))
            -> std::enable_if_t<IsBidirectional, iterator>
        {
            auto tmp = *this;
            --*this;
            return tmp;
        }

        constexpr bool operator==(const iterator &other) const
            noexcept(impl::traits::nothrow_comparable<IteratorType>)
        {
            return it == other.it;
        }

        constexpr bool operator!=(const iterator &other) const
            noexcept(noexcept(other == other))
        {
            return !(*this == other);
        }

        constexpr reference operator*() const
        {
            return reference(it->first, *it->second);
        }

        constexpr pointer operator->() const
        {
            val.emplace(it->first, *it->second);
            return &*val;
        }

    private:
        IteratorType it;
        mutable std::optional<value_type> val{};
    };

private:
    using BaseIterator = decltype(std::declval<ForwardMap>().begin());
    static constexpr bool iterator_ctor_nothrow =
        std::is_nothrow_constructible_v<iterator, BaseIterator>;

public:
    bimap() : map(), inverseAccess(*this) {}

    template <typename InputIt>
    bimap(InputIt start, InputIt end) : bimap()
    {
        while (start != end) {
            emplace(*start);
            ++start;
        }
    }

    bimap(std::initializer_list<std::pair<ForwardKey, InverseKey>> init)
        : bimap(init.begin(), init.end())
    {
    }

    bimap(const bimap &other) : bimap()
    {
        for (const auto &valuePair : other) {
            emplace(valuePair);
        }
    }

    void swap(bimap &other) noexcept(std::is_nothrow_swappable_v<ForwardMap> &&
                                     std::is_nothrow_swappable_v<InverseMap>)
    {
        std::swap(this->map, other.map);
        std::swap(this->inverseAccess->map, other.inverseAccess->map);
    }

    bimap(bimap &&other) : bimap() { swap(other); }

    bimap &operator=(bimap other) noexcept(
        noexcept(std::declval<bimap>().swap(other)))
    {
        swap(other);
        return *this;
    }

    ~bimap() = default;

    template <typename... Args>
    auto emplace(Args &&...args) -> std::pair<iterator, bool>
    {
        std::pair<ForwardKey, InverseKey> tmp(std::forward<Args>(args)...);
        if constexpr (!impl::traits::is_multimap_v<ForwardMap>) {
            auto res = find(tmp.first);
            if (res != end()) {
                return {res, false};
            }
        }

        if constexpr (!impl::traits::is_multimap_v<InverseMap>) {
            auto invRes = inverse().find(tmp.second);
            if (invRes != inverse().end()) {
                return {find(invRes->second), false};
            }
        }

        auto it = impl::get_first(map.emplace(std::move(tmp.first), nullptr));
        auto invIt = impl::get_first(
            inverseAccess->map.emplace(std::move(tmp.second), &it->first));
        it->second = &invIt->first;
        return {iterator(it), true};
    }

    [[nodiscard]] auto size() const
        noexcept(noexcept(std::declval<ForwardMap>().size()))
    {
        return map.size();
    }

    [[nodiscard]] bool empty() const
        noexcept(noexcept(std::declval<ForwardMap>().empty()))
    {
        return map.empty();
    }

    constexpr auto inverse() noexcept -> InverseBiMap &
    {
        return *inverseAccess;
    }

    constexpr auto inverse() const noexcept -> const InverseBiMap &
    {
        return *inverseAccess;
    }

    iterator begin() const
        noexcept(noexcept(std::declval<ForwardMap>().begin()) &&
                 iterator_ctor_nothrow)
    {
        return iterator(map.begin());
    }

    iterator end() const noexcept(noexcept(std::declval<ForwardMap>().end()) &&
                                  iterator_ctor_nothrow)
    {
        return iterator(map.end());
    }

    iterator find(const ForwardKey &key) const
        noexcept(noexcept(std::declval<ForwardMap>().find(key)) &&
                 iterator_ctor_nothrow)
    {
        return iterator(map.find(key));
    }

    template <REQUIRES_THAT(ForwardMap, std::declval<_T_>().lower_bound(
                                            std::declval<ForwardKey>()))>
    auto lower_bound(const ForwardKey &key) const
        noexcept(noexcept(std::declval<ForwardMap>().lower_bound(key)) &&
                 iterator_ctor_nothrow) -> iterator
    {
        return iterator(map.lower_bound(key));
    }

    template <REQUIRES_THAT(ForwardMap, std::declval<_T_>().upper_bound(
                                            std::declval<ForwardKey>()))>
    auto upper_bound(const ForwardKey &key) const
        noexcept(noexcept(std::declval<ForwardMap>().upper_bound(key)) &&
                 iterator_ctor_nothrow) -> iterator
    {
        return iterator(map.upper_bound(key));
    }

    auto equal_range(const ForwardKey &key) const
        noexcept(noexcept(std::declval<ForwardMap>().equal_range(key)) &&
                 iterator_ctor_nothrow) -> std::pair<iterator, iterator>
    {
        auto [first, last] = map.equal_range(key);
        return {iterator(first), iterator(last)};
    }

    iterator erase(iterator pos)
    {
        if (pos == end()) {
            return pos;
        }

        if constexpr (impl::traits::is_multimap_v<InverseMap>) {
            auto [curr, end] = inverse().equal_range(pos->second);
            while (curr != end && &curr->first != pos.it->second.get()) {
                ++curr;
            }

            assert(curr != end);
            inverseAccess->map.erase(curr.it);
        }
        else {
            inverseAccess->map.erase(inverseAccess->map.find(pos->second));
        }

        return iterator(map.erase(pos.it));
    }

    std::size_t erase(const ForwardKey &key)
    {
        auto [curr, last] = equal_range(key);
        std::size_t numErased = 0;
        while (curr != last) {
            curr = erase(curr);
            ++numErased;
        }

        return numErased;
    }

    iterator erase(iterator first, iterator last)
    {
        while (first != last && first != end()) {
            first = erase(first);
        }

        return first;
    }

    bool operator==(const bimap &other) const
        noexcept(impl::traits::nothrow_comparable<ForwardMap> &&
                 impl::traits::nothrow_comparable<InverseMap>)
    {
        return map == other.map &&
               inverseAccess->map == other.inverseAccess->map;
    }

    bool operator!=(const bimap &other) const noexcept(noexcept(other == other))
    {
        return !(*this == other);
    }

    void clear() noexcept(noexcept(std::declval<ForwardMap>().clear()) &&
                          noexcept(std::declval<InverseMap>().clear()))
    {
        map.clear();
        inverseAccess->map.clear();
    }

    bool contains(const ForwardKey &key) const
        noexcept(noexcept(std::declval<bimap>().find(key)) &&
                 noexcept(std::declval<iterator>() != std::declval<iterator>()))
    {
        return find(key) != end();
    }

    template <bool UniqueKeys = !impl::traits::is_multimap_v<ForwardMap>>
    auto at(const ForwardKey &key) const
        -> std::enable_if_t<UniqueKeys, const InverseKey &>
    {
        auto res = find(key);
        if (res == end()) {
            throw std::out_of_range("bidirectional map key not found");
        }

        return res->second;
    }

private:
    ForwardMap map;
    InversBiMapPtr inverseAccess;
};

template <typename ForwardKey, typename InverseKey,
          template <typename...> typename ForwardMapType = std::unordered_map,
          template <typename...> typename InverseMapType = std::unordered_map>
void swap(bimap<ForwardKey, InverseKey, ForwardMapType, InverseMapType> &lhs,
          bimap<ForwardKey, InverseKey, ForwardMapType, InverseMapType>
              &rhs) noexcept(noexcept(lhs.swap(rhs)))
{
    lhs.swap(rhs);
}

} // namespace tl
