#pragma once

#include <functional>
#include <utility>

namespace std {

namespace {

// Learn from Boost
template <class T>
inline void hashCombine(const T& v, std::size_t* seed)
{
    std::hash<T> hasher;
    *seed ^= hasher(v) + 0x9e3779b9 + (*seed << 6) + (*seed >> 2);
}

} // namespace

// Hash for std::pair
template <typename T1, typename T2>
struct hash<std::pair<T1, T2>>
{
public:
    size_t operator()(const std::pair<T1, T2>& e) const
    {
        size_t seed = 0;
        hashCombine(e.first, &seed);
        hashCombine(e.second, &seed);
        return seed;
    }
};

} // namespace std
