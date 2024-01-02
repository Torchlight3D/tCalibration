#pragma once

#include <algorithm>
#include <vector>

#include "ax_core_global.h"

namespace thoht {

namespace utils {

template <typename T, size_t N>
inline constexpr size_t ArraySize(T (&arr)[N])
{
    return N;
}

template <typename T>
void Erase(std::vector<T>& vec, size_t index)
{
    auto it = vec.begin();
    std::advance(it, index);
    vec.erase(it);
}

template <typename T>
T FindMedian(std::vector<T>& vec)
{
    const auto n = vec.size();
    const auto mid = n / 2;
    std::nth_element(vec.begin(), vec.begin() + mid, vec.end());

    const auto& median = vec[mid];
    if (n & 1) {
        return median;
    }

    auto max = std::max_element(vec.begin(), vec.begin() + mid);
    return (*max + median) / 2.;
}

template <typename T>
inline bool Contains(const std::vector<T>& vec, const T& val)
{
    return std::any_of(vec.cbegin(), vec.cend(),
                       [&val](const T& v) { return v == val; });
}

template <typename T>
inline int FindPos(const std::vector<T>& vec, const T& val)
{
    const auto found = std::find(vec.cbegin(), vec.cend(), val);
    if (found == vec.cend()) {
        return -1;
    }
    return std::distance(vec.cbegin(), found);
}

template <class Map_t>
using MapKey = typename Map_t::value_type::first_type;

template <class Map_t>
using MapValue = typename Map_t::value_type::second_type;

template <class Map_t>
MapValue<Map_t>& FindOrDie(Map_t& map, const MapKey<Map_t>& key)
{
    auto found = map.find(key);
    CHECK(found != map.end()) << "Map key not found: " << key;
    return found->second;
}

template <class Map_t>
const MapValue<Map_t>& FindOrDie(const Map_t& map, const MapKey<Map_t>& key)
{
    auto found = map.find(key);
    CHECK(found != map.end()) << "Map key not found: " << key;
    return found->second;
}

template <class Map_t>
MapValue<Map_t>& FindOrDieNoPrint(Map_t& map, const MapKey<Map_t>& key)
{
    auto found = map.find(key);
    CHECK(found != map.end());
    return found->second;
}

template <class Map_t>
const MapValue<Map_t>& FindOrDieNoPrint(const Map_t& map,
                                        const MapKey<Map_t>& key)
{
    auto found = map.find(key);
    CHECK(found != map.end());
    return found->second;
}

template <class Map_t>
MapValue<Map_t>* FindOrNull(Map_t& map, const MapKey<Map_t>& key)
{
    auto it = map.find(key);
    if (it == map.end()) {
        return nullptr;
    }
    return &it->second;
}

template <class Map_t>
const MapValue<Map_t>* FindOrNull(const Map_t& map, const MapKey<Map_t>& key)
{
    auto it = map.find(key);
    if (it == map.end()) {
        return nullptr;
    }
    return &it->second;
}

template <class Map_t>
const MapValue<Map_t>& FindWithDefault(const Map_t& map,
                                       const MapKey<Map_t>& key,
                                       const MapValue<Map_t>& value)
{
    auto it = map.find(key);
    if (it == map.end()) {
        return value;
    }
    return it->second;
}

// NOTE: hide after Cpp20
template <class Map_t, class Key_t>
bool ContainsKey(const Map_t& map, const Key_t& key)
{
    return map.find(key) != map.end();
}

template <typename Map_t>
void ContainerIntersection(const Map_t& map1, const Map_t& map2, Map_t* out)
{
    if (map2.size() < map1.size()) {
        return ContainerIntersection(map2, map1, out);
    }

    for (const auto& entry : map1) {
        if (ContainsKey(map2, entry)) {
            out->insert(entry);
        }
    }
}

} // namespace utils

} // namespace thoht
