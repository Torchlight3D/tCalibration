#include <random>
#include <set>
#include <unordered_set>

#include <gtest/gtest.h>

#include <tCore/ContainerUtils>

using namespace tl;

TEST(Container, ContatinerIntersectionSet)
{
    std::random_device rd{};
    std::mt19937 rng{rd()};
    std::uniform_int_distribution<> dist{1, 1000};

    std::set<int> common;
    while (common.size() < 10) {
        common.insert(dist(rng));
    }

    auto set1 = common;
    while (set1.size() < 8) {
        set1.insert(dist(rng));
    }

    auto set2 = common;
    while (set2.size() < 11) {
        set2.insert(dist(rng));
    }

    const auto intersection = con::ContainerIntersection(set1, set2);
    EXPECT_EQ(intersection, common);
}

TEST(Container, ContatinerIntersectionUnorderedSet)
{
    std::random_device rd{};
    std::mt19937 rng{rd()};
    std::uniform_int_distribution<> dist{1, 1000};

    std::unordered_set<int> common;
    while (common.size() < 10) {
        common.insert(dist(rng));
    }

    auto set1 = common;
    while (set1.size() < 8) {
        set1.insert(dist(rng));
    }

    auto set2 = common;
    while (set2.size() < 11) {
        set2.insert(dist(rng));
    }

    const auto intersection = con::ContainerIntersection(set1, set2);
    EXPECT_EQ(intersection, common);
}
