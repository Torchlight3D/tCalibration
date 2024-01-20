#pragma once

#include <type_traits>

namespace tl {

template <typename Enum_t>
struct CustomFlags
{
    static constexpr bool enable{false};
};

#define MAKE_FLAGS(name)                    \
    template <>                             \
    struct CustomFlags<name>                \
    {                                       \
        static constexpr bool enable{true}; \
    };

template <typename Enum_t>
using enum_type =
    typename std::enable_if_t<CustomFlags<Enum_t>::enable, Enum_t>;

template <typename Enum_t>
constexpr enum_type<Enum_t> operator|(Enum_t lhs, Enum_t rhs)
{
    using inner = typename std::underlying_type_t<Enum_t>;
    return static_cast<Enum_t>(static_cast<inner>(lhs) |
                               static_cast<inner>(rhs));
}

template <typename Enum_t>
constexpr enum_type<Enum_t> operator&(Enum_t lhs, Enum_t rhs)
{
    using inner = typename std::underlying_type_t<Enum_t>;
    return static_cast<Enum_t>(static_cast<inner>(lhs) &
                               static_cast<inner>(rhs));
}

template <typename Enum_t>
constexpr enum_type<Enum_t> operator^(Enum_t lhs, Enum_t rhs)
{
    using inner = typename std::underlying_type_t<Enum_t>;
    return static_cast<Enum_t>(static_cast<inner>(lhs) ^
                               static_cast<inner>(rhs));
}

template <typename Enum_t>
constexpr enum_type<Enum_t> operator~(Enum_t lhs)
{
    using inner = typename std::underlying_type_t<Enum_t>;
    return static_cast<Enum_t>(~static_cast<inner>(lhs));
}

template <typename Enum_t>
using enum_type_ref =
    typename std::enable_if_t<CustomFlags<Enum_t>::enable, Enum_t&>;

template <typename Enum_t>
constexpr enum_type_ref<Enum_t> operator|=(Enum_t& lhs, Enum_t rhs)
{
    using inner = typename std::underlying_type_t<Enum_t>;
    lhs =
        static_cast<Enum_t>(static_cast<inner>(lhs) | static_cast<inner>(rhs));
    return lhs;
}

template <typename Enum_t>
constexpr enum_type_ref<Enum_t> operator&=(Enum_t& lhs, Enum_t rhs)
{
    using inner = typename std::underlying_type_t<Enum_t>;
    lhs =
        static_cast<Enum_t>(static_cast<inner>(lhs) & static_cast<inner>(rhs));
    return lhs;
}

template <typename Enum_t>
constexpr enum_type_ref<Enum_t> operator^=(Enum_t& lhs, Enum_t rhs)
{
    using inner = typename std::underlying_type_t<Enum_t>;
    lhs =
        static_cast<Enum_t>(static_cast<inner>(lhs) ^ static_cast<inner>(rhs));
    return lhs;
}

} // namespace tl
