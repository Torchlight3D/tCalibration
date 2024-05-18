#include <map>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <gtest/gtest.h>

#include <tCore/Bimap>
#include <tCore/Global>

using namespace tl;

class MustNotCopy
{
public:
    MustNotCopy() = default;
    MustNotCopy(std::string string) : s(std::move(string)) {}
    MustNotCopy(const char *chars) : MustNotCopy(std::string(chars)) {}

    MustNotCopy(const MustNotCopy &other) : s(other.s)
    {
        // Hack because FAIL() macro is a non-void return statement
        EXPECT_TRUE(false) << "Copy occurred";
    }
    MustNotCopy(MustNotCopy &&) = default;
    MustNotCopy &operator=(const MustNotCopy &) = default;
    MustNotCopy &operator=(MustNotCopy &&) = default;

    operator std::string() { return s; }

    bool operator==(const MustNotCopy &other) const { return s == other.s; }
    bool operator!=(const MustNotCopy &other) const
    {
        return !(*this == other);
    }

    struct Hash
    {
        std::size_t operator()(const MustNotCopy &m) const
        {
            return std::hash<std::string>()(m.s);
        }
    };

    std::string s;
};

template <typename T, typename U>
struct BadIterator
{
    using iterator_category = std::bidirectional_iterator_tag;
    using value_type = std::pair<T, U>;
    using reference = std::add_lvalue_reference_t<std::add_const_t<value_type>>;
    using pointer = std::add_pointer_t<value_type>;
    using difference_type = std::size_t;

    BadIterator &operator++()
    {
        throw std::runtime_error("This exception should not appear");
    }

    BadIterator operator++(int)
    {
        throw std::runtime_error("This exception should not appear");
    }

    BadIterator &operator--()
    {
        throw std::runtime_error("This exception should not appear");
    }

    BadIterator operator--(int)
    {
        throw std::runtime_error("This exception should not appear");
    }

    bool operator==(const BadIterator &) const
    {
        throw std::runtime_error("This exception should not appear");
    }

    bool operator!=(const BadIterator &) const
    {
        throw std::runtime_error("This exception should not appear");
    }

    reference operator*() const noexcept
    {
        throw std::runtime_error("This exception should not appear");
    }

    pointer operator->() const
    {
        throw std::runtime_error("This exception should not appear");
    }
};

template <typename T, typename U>
struct BadContainer
{
    auto begin() const { return BadIterator<T, U>{}; }
};

template <typename _Key, typename _Val>
using NotCopyMap = std::unordered_map<_Key, _Val, MustNotCopy::Hash>;

template <typename T, typename U>
struct BadMap
{
    std::size_t size() const
    {
        throw std::runtime_error("This exception should not appear");
    }

    bool empty() const
    {
        throw std::runtime_error("This exception should not appear");
    }

    void reserve(std::size_t)
    {
        throw std::runtime_error("This exception should not appear");
    }

    int *begin() const
    {
        throw std::runtime_error("This exception should not appear");
    }

    int *end() const
    {
        throw std::runtime_error("This exception should not appear");
    }

    int *find(const T &) const
    {
        throw std::runtime_error("This exception should not appear");
    }

    void clear()
    {
        throw std::runtime_error("This exception should not appear");
    }
};

template <typename It, typename _Key, typename _Val>
void checkIterator(It it, const _Key &first, const _Val &second)
{
    EXPECT_EQ(it->first, first);
    EXPECT_EQ(it->second, second);
}

TEST(BidirectionalMap, DefaultCTOR)
{
    bimap<std::string, int> test;
    EXPECT_TRUE(test.empty());
    EXPECT_EQ(test.size(), 0);
}

TEST(BidirectionalMap, InitializerCTOR)
{
    bimap<std::string, int> test{{"Test", 1}, {"SecondItem", 2}};
    EXPECT_FALSE(test.empty());
    EXPECT_EQ(test.size(), 2);
    checkIterator(test.find("Test"), "Test", 1);
    checkIterator(test.find("SecondItem"), "SecondItem", 2);
}

TEST(BidirectionalMap, IteratorCTOR)
{
    std::unordered_map<std::string, int> tmp = {{"Test", 1}, {"SecondItem", 2}};
    auto tmpCopy = tmp;
    bimap<std::string, int> test(tmp.begin(), tmp.end());
    EXPECT_EQ(tmp, tmpCopy);
    EXPECT_FALSE(test.empty());
    EXPECT_EQ(test.size(), 2);
    checkIterator(test.find("Test"), "Test", 1);
    checkIterator(test.find("SecondItem"), "SecondItem", 2);
}

TEST(BidirectionalMap, Emplace)
{
    bimap<std::string, int> test;
    auto [it, _] = test.emplace("Test", 123);
    EXPECT_EQ(test.size(), 1);
    EXPECT_FALSE(test.empty());
    checkIterator(it, "Test", 123);
}

TEST(BidirectionalMap, Uniqueness)
{
    bimap<std::string, int> test;
    auto [it, inserted] = test.emplace("Test", 123);
    EXPECT_TRUE(inserted);
    checkIterator(it, "Test", 123);
    std::tie(it, inserted) = test.emplace("NewItem", 456);
    EXPECT_TRUE(inserted);
    checkIterator(it, "NewItem", 456);
    std::tie(it, inserted) = test.emplace("Test", 123);
    EXPECT_FALSE(inserted);
    checkIterator(it, "Test", 123);
    std::tie(it, inserted) = test.emplace("Test", 765);
    EXPECT_FALSE(inserted);
    checkIterator(it, "Test", 123);
    std::tie(it, inserted) = test.emplace("EqualInverseKey", 456);
    EXPECT_FALSE(inserted);
    checkIterator(it, "NewItem", 456);
}

TEST(BidirectionalMap, Find)
{
    bimap<std::string, int> test;
    test.emplace("Test", 123);
    test.emplace("NewItem", 456);
    auto it = test.find("Test");
    checkIterator(it, "Test", 123);
    it = test.find("Stuff");
    EXPECT_EQ(it, test.end());
}

TEST(BidirectionalMap, Contains)
{
    bimap<std::string, int> test{{"Test", 123}, {"NewItem", 456}};
    EXPECT_TRUE(test.contains("Test"));
    EXPECT_TRUE(test.contains("NewItem"));
    EXPECT_FALSE(test.contains("abc"));
}

TEST(BidirectionalMap, EraseByIterator)
{
    bimap<std::string, int> test;
    test.emplace("Test", 123);
    test.emplace("NewItem", 456);
    test.erase(test.find("NewItem"));
    EXPECT_EQ(test.size(), 1);
    EXPECT_EQ(test.find("NewItem"), test.end());
    checkIterator(test.find("Test"), "Test", 123);
    EXPECT_EQ(test.erase(test.end()), test.end());
    EXPECT_EQ(test.size(), 1);
    EXPECT_EQ(test.find("NewItem"), test.end());
}

TEST(BidirectionalMap, EraseByKey)
{
    bimap<std::string, int> test;
    test.emplace("Test", 123);
    test.emplace("NewItem", 456);
    EXPECT_EQ(test.erase("NewItem"), 1);
    EXPECT_EQ(test.size(), 1);
    EXPECT_EQ(test.find("NewItem"), test.end());
    checkIterator(test.find("Test"), "Test", 123);
    EXPECT_EQ(test.erase("Stuff"), 0);
    EXPECT_EQ(test.size(), 1);
    checkIterator(test.find("Test"), "Test", 123);
}

TEST(BidirectionalMap, EraseByRange)
{
    bimap<std::string, int, std::map> test{{"Item1", 123},  {"Item2", 456},
                                           {"Item3", 789},  {"Item4", 1123},
                                           {"Item5", 1456}, {"Item6", 1789}};
    auto start = test.begin();
    ++start;
    auto end = start;
    ++end;
    ++end;
    ++end, ++end;
    auto res = test.erase(start, end);
    EXPECT_EQ(test.size(), 2);
    checkIterator(test.begin(), "Item1", 123);
    checkIterator(res, "Item6", 1789);
}

TEST(BidirectionalMap, Iterate)
{
    bimap<std::string, int, std::map> test{
        {"Item1", 123}, {"Item2", 456}, {"Item3", 789}};
    std::unordered_map<std::string, int> lookup = {
        {"Item1", 123}, {"Item2", 456}, {"Item3", 789}};
    for (auto it = test.begin(); it != test.end(); ++it) {
        auto lookupRes = lookup.find(it->first);
        checkIterator(it, lookupRes->first, lookupRes->second);
        lookup.erase(it->first);
    }

    EXPECT_TRUE(lookup.empty());
}

TEST(BidirectionalMap, IterateEmpty)
{
    bimap<std::string, int> test;
    EXPECT_EQ(test.begin(), test.end());
}

TEST(BidirectionalMap, IterateWithErase)
{
    bimap<std::string, int, std::map> test = {
        {"Item1", 123}, {"Item2", 456}, {"Item3", 789}};
    std::unordered_map<std::string, int> lookup = {{"Item1", 123},
                                                   {"Item3", 789}};
    for (auto it = test.begin(); it != test.end(); ++it) {
        if (it->first == "Item2") {
            it = test.erase(it);
        }

        auto lookupRes = lookup.find(it->first);
        checkIterator(it, lookupRes->first, lookupRes->second);
        lookup.erase(it->first);
    }

    EXPECT_TRUE(lookup.empty());
    EXPECT_EQ(test.size(), 2);
}

TEST(BidirectionalMap, BidirectionalIterate)
{
    bimap<std::string, int, std::map> test{
        {"Item1", 123}, {"Item2", 456}, {"Item3", 789}};
    const auto start = test.begin();
    auto curr = start;
    ++curr;
    checkIterator(curr++, "Item2", 456);
    EXPECT_EQ(++curr, test.end());
    --curr;
    checkIterator(--curr, "Item2", 456);
    checkIterator(curr--, "Item2", 456);
}

TEST(BidirectionalMap, Comparison)
{
    bimap<std::string, int> original{
        {"Test", 123}, {"NewItem", 456}, {"Stuff", 789}};
    bimap<std::string, int> test1{
        {"Test", 123}, {"NewItem", 456}, {"Stuff", 789}};
    bimap<std::string, int> test2{{"Test", 123}, {"Stuff", 789}};
    bimap<std::string, int> test3{
        {"Test", 0}, {"NewItem", 456}, {"Stuff", 789}};
    bimap<std::string, int> test4{
        {"Testing", 123}, {"NewItem", 456}, {"Stuff", 789}};
    EXPECT_EQ(original, test1);
    EXPECT_NE(original, test2);
    EXPECT_NE(original, test3);
    EXPECT_NE(original, test4);
}

TEST(BidirectionalMap, CopyCTOR)
{
    bimap<std::string, int> original{
        {"Test", 123}, {"NewItem", 456}, {"Stuff", 789}};
    auto copy = original;
    EXPECT_EQ(original, copy);
    original.emplace("AddStuff", 17);
    EXPECT_EQ(copy.size(), 3);
    EXPECT_EQ(copy.find("AddStuff"), copy.end());
    copy.emplace("CopyItem", 17);
    EXPECT_EQ(original.size(), 4);
    EXPECT_EQ(original.find("CopyItem"), original.end());
}

TEST(BidirectionalMap, copy_ctor_elements)
{
    bimap<std::string, int> original{
        {"Test", 123}, {"NewItem", 456}, {"Stuff", 789}};
    auto copy = original;
    auto origIt = original.begin();
    auto copyIt = copy.begin();
    while (origIt != original.end()) {
        EXPECT_NE(&origIt->first, &copyIt->first);
        EXPECT_NE(&origIt->second, &copyIt->second);
        ++origIt;
        ++copyIt;
    }
}

TEST(BidirectionalMap, MoveCTOR)
{
    bimap<std::string, int> original{
        {"Test", 123}, {"NewItem", 456}, {"Stuff", 789}};
    auto copy = original;
    auto moved = std::move(original);
    EXPECT_EQ(moved, copy);
    moved.emplace("AnotherItem", 17);
    EXPECT_EQ(moved.size(), 4);
    checkIterator(moved.find("AnotherItem"), "AnotherItem", 17);
}

TEST(BidirectionalMap, Assign)
{
    bimap<std::string, int> original{
        {"Test", 123}, {"NewItem", 456}, {"Stuff", 789}};
    bimap<std::string, int> overwritten = {{"abc", 1}};
    overwritten = original;
    EXPECT_EQ(overwritten, original);
    original.emplace("AddStuff", 17);
    EXPECT_EQ(overwritten.size(), 3);
    EXPECT_EQ(overwritten.find("AddStuff"), overwritten.end());
    auto copy = original;
    overwritten = std::move(original);
    EXPECT_EQ(overwritten, copy);
}

TEST(BidirectionalMap, UseAfterMove)
{
    bimap<std::string, int> original{
        {"Test", 123}, {"NewItem", 456}, {"Stuff", 789}};
    auto moved = std::move(original);
    EXPECT_TRUE(original.empty());
    EXPECT_EQ(original.begin(), original.end());
    EXPECT_TRUE(original.inverse().empty());
    EXPECT_EQ(original.inverse().begin(), original.inverse().end());
    original.emplace("Test", 123);
    original.inverse().emplace(456, "NewItem");
    EXPECT_EQ(original.size(), 2);
    EXPECT_EQ(original.at("Test"), 123);
    EXPECT_EQ(original.at("NewItem"), 456);
}

TEST(BidirectionalMap, ReturnInverse)
{
    auto generator = []() -> bimap<int, std::string> {
        bimap<std::string, int> map = {
            {"Test", 123}, {"NewItem", 456}, {"Stuff", 789}};
        return std::move(map.inverse());
    };

    auto test = generator();
    EXPECT_EQ(test.size(), 3);
    EXPECT_EQ(test.at(123), "Test");
    test.emplace(1, "one");
    EXPECT_EQ(test.at(1), "one");
    EXPECT_EQ(test.size(), 4);
}

TEST(BidirectionalMap, Swap)
{
    bimap<std::string, int> map1{
        {"Test", 123}, {"NewItem", 456}, {"Stuff", 789}};
    bimap<std::string, int> map2{{"One", 1}, {"Two", 2}};
    std::swap(map1, map2);
    EXPECT_EQ(map1.size(), 2);
    EXPECT_EQ(map2.size(), 3);
    EXPECT_EQ(map1.at("One"), 1);
    EXPECT_EQ(map1.at("Two"), 2);
    EXPECT_EQ(map1.inverse().at(1), "One");
    EXPECT_EQ(map1.inverse().at(2), "Two");
    EXPECT_EQ(map2.at("Test"), 123);
    EXPECT_EQ(map2.at("NewItem"), 456);
    EXPECT_EQ(map2.at("Stuff"), 789);
    EXPECT_EQ(map2.inverse().at(123), "Test");
    EXPECT_EQ(map2.inverse().at(456), "NewItem");
    EXPECT_EQ(map2.inverse().at(789), "Stuff");
    EXPECT_EQ(map1, map1.inverse().inverse());
    EXPECT_EQ(map2, map2.inverse().inverse());
}

TEST(BidirectionalMap, MoveSwapBackAndForth)
{
    bimap<std::string, int> original{
        {"Test", 123}, {"NewItem", 456}, {"Stuff", 789}};
    auto moved = std::move(original);
    moved.inverse().emplace(0, "zero");
    original = std::move(moved);
    EXPECT_EQ(original.size(), 4);
    EXPECT_EQ(original.at("zero"), 0);
}

TEST(BidirectionalMap, Clear)
{
    bimap<std::string, int> test{{"Test", 123}};
    test.clear();
    EXPECT_TRUE(test.empty());
    EXPECT_EQ(test.size(), 0);
    EXPECT_EQ(test.find("Test"), test.end());
}

TEST(BidirectionalMap, InverseAccess)
{
    bimap<std::string, int> test{
        {"Test", 123}, {"NewItem", 456}, {"Stuff", 789}};
    const auto &inverse = test.inverse();
    EXPECT_EQ(inverse.size(), 3);
    checkIterator(inverse.find(123), 123, "Test");
    checkIterator(inverse.find(456), 456, "NewItem");
    checkIterator(inverse.find(789), 789, "Stuff");
}

TEST(BidirectionalMap, InverseAccessEmplace)
{
    bimap<std::string, int> test{
        {"Test", 123}, {"NewItem", 456}, {"Stuff", 789}};
    auto &inverse = test.inverse();
    inverse.emplace(17, "Inverse");
    EXPECT_EQ(inverse.size(), 4);
    EXPECT_EQ(test.size(), 4);
    checkIterator(test.find("Inverse"), "Inverse", 17);
    checkIterator(inverse.find(17), 17, "Inverse");
    auto [it, inserted] = inverse.emplace(123, "bla");
    EXPECT_EQ(inverse.size(), 4);
    EXPECT_EQ(test.size(), 4);
    EXPECT_EQ(test.find("bla"), test.end());
    EXPECT_FALSE(inserted);
    checkIterator(it, 123, "Test");
}

TEST(BidirectionalMap, InverseAccessIdentity)
{
    bimap<std::string, int> test{
        {"Test", 123}, {"NewItem", 456}, {"Stuff", 789}};
    auto &same = test.inverse().inverse();
    EXPECT_EQ(test, same);
    same.emplace("abc", 17);
    EXPECT_EQ(test.size(), 4);
    EXPECT_EQ(same.size(), 4);
    checkIterator(test.find("abc"), "abc", 17);
    checkIterator(same.find("abc"), "abc", 17);
}

TEST(BidirectionalMap, InverseAccessClear)
{
    bimap<std::string, int> test{{"Test", 123}};
    auto &inverse = test.inverse();
    test.clear();
    EXPECT_TRUE(inverse.empty());
    EXPECT_EQ(inverse.find(123), inverse.end());
    inverse.emplace(123, "Test");
    inverse.clear();
    EXPECT_TRUE(inverse.empty());
    EXPECT_TRUE(test.empty());
    EXPECT_EQ(inverse.find(123), inverse.end());
    EXPECT_EQ(test.find("Test"), test.end());
}

TEST(BidirectionalMap, InverseAccessErase)
{
    bimap<std::string, int> test{
        {"Test", 123}, {"NewItem", 456}, {"AnotherItem", 789}};
    auto &inverse = test.inverse();
    test.erase("NewItem");
    EXPECT_EQ(inverse.size(), 2);
    EXPECT_EQ(inverse.find(456), inverse.end());
    inverse.erase(123);
    EXPECT_EQ(test.size(), 1);
    EXPECT_EQ(test.find("Test"), test.end());
}

TEST(BidirectionalMap, InverseAccessEmplaceAfterMoved)
{
    bimap<std::string, int> original{
        {"Test", 123}, {"NewItem", 456}, {"Stuff", 789}};
    auto moved = std::move(original);
    moved.inverse().emplace(17, "AnotherItem");
    EXPECT_EQ(moved.inverse().size(), 4);
    checkIterator(moved.inverse().find(17), 17, "AnotherItem");
    checkIterator(moved.find("AnotherItem"), "AnotherItem", 17);
}

TEST(BidirectionalMap, CopyInverse)
{
    bimap<std::string, int> original{
        {"Test", 123}, {"NewItem", 456}, {"Stuff", 789}};
    auto copy = original.inverse();
    EXPECT_EQ(copy, original.inverse());
    original.emplace("AddStuff", 17);
    EXPECT_EQ(copy.size(), 3);
    EXPECT_EQ(copy.find(17), copy.end());
    copy.emplace(18, "NewCopyItem");
    EXPECT_EQ(original.size(), 4);
    EXPECT_EQ(original.find("NewCopyItem"), original.end());
    copy.inverse().erase("Test");
    EXPECT_EQ(copy.find(123), copy.end());
    checkIterator(original.find("Test"), "Test", 123);
}

TEST(BidirectionalMap, MoveInverse)
{
    bimap<std::string, int> original{
        {"Test", 123}, {"NewItem", 456}, {"Stuff", 789}};
    auto moved = std::move(original.inverse());
    moved.emplace(18, "NewMoveItem");
    checkIterator(moved.find(18), 18, "NewMoveItem");
    moved.inverse().erase("Test");
    EXPECT_EQ(moved.find(123), moved.end());
    checkIterator(moved.find(456), 456, "NewItem");
}

TEST(BidirectionalMap, ZeroCopy)
{
    bimap<MustNotCopy, int, NotCopyMap> test;
    test.emplace("Test1", 1);
    test.emplace("Test2", 2);
    test.emplace("Test3", 3);
    auto moved = std::move(test);
    auto it = moved.begin();
    while (it != moved.end()) {
        ++it;
    }

    std::vector<std::string> strings;
    for (const auto &[mnc, _] : moved) {
        strings.emplace_back(mnc.s);
    }

    for (const auto &[_, mnc] : moved.inverse()) {
        strings.emplace_back(mnc.s);
    }
}

TEST(BidirectionalMap, AccessByIndex)
{
    bimap<std::string, int> test{
        {"Test", 123}, {"NewItem", 456}, {"Stuff", 789}};
    EXPECT_EQ(test.at("Test"), 123);
    EXPECT_EQ(test.at("Stuff"), 789);
    EXPECT_EQ(test.inverse().at(456), "NewItem");
    EXPECT_THROW(test.at("NotIncluded"), std::out_of_range);
    EXPECT_THROW(test.inverse().at(0), std::out_of_range);
}

TEST(BidirectionalMap, NoexceptIterator)
{
    bimap<std::string, int, std::map> test;
    auto it = test.begin();
    EXPECT_TRUE(noexcept(++it));
    EXPECT_TRUE(noexcept(it++));
    EXPECT_TRUE(noexcept(--it));
    EXPECT_TRUE(noexcept(it--));
    EXPECT_TRUE(noexcept(it == it));
    EXPECT_TRUE(noexcept(it != it));
    EXPECT_TRUE(std::is_nothrow_copy_constructible_v<decltype(it)>);
    EXPECT_TRUE(std::is_nothrow_copy_assignable_v<decltype(it)>);
    EXPECT_TRUE(std::is_nothrow_move_constructible_v<decltype(it)>);
    EXPECT_TRUE(std::is_nothrow_move_assignable_v<decltype(it)>);
}

TEST(BidirectionalMap, ThrowIterator)
{
    BadIterator<std::string, impl::Surrogate<const int>> baseIt;
    bimap<std::string, int, BadContainer>::iterator badIt(baseIt);
    EXPECT_THROW(++badIt, std::runtime_error);
    EXPECT_THROW(badIt++, std::runtime_error);
    EXPECT_THROW(--badIt, std::runtime_error);
    EXPECT_THROW(badIt--, std::runtime_error);
    EXPECT_THROW(UNUSED(badIt == badIt), std::runtime_error);
    EXPECT_THROW(UNUSED(badIt != badIt), std::runtime_error);
    EXPECT_THROW(*badIt, std::runtime_error);
    EXPECT_THROW(UNUSED(badIt->first), std::runtime_error);
    EXPECT_THROW(UNUSED(badIt->second), std::runtime_error);
}

TEST(BidirectionalMap, ThrowBadContainer)
{
    bimap<std::string, int, BadMap> test;
    EXPECT_THROW(UNUSED(test.size()), std::runtime_error);
    EXPECT_THROW(UNUSED(test.empty()), std::runtime_error);
    EXPECT_THROW(test.begin(), std::runtime_error);
    EXPECT_THROW(test.end(), std::runtime_error);
    EXPECT_THROW(test.find(""), std::runtime_error);
    EXPECT_THROW(test.contains(""), std::runtime_error);
    EXPECT_THROW(test.clear(), std::runtime_error);
}

TEST(BidirectionalMap, MultiMap)
{
    bimap<std::string, int, std::multimap> test{
        {"Test", 123}, {"NewItem", 456}, {"Stuff", 789}};
    auto [it, inserted] = test.emplace("Hello", 123);
    EXPECT_EQ(test.size(), 3);
    EXPECT_FALSE(inserted);
    EXPECT_EQ(it, test.find("Test"));
    std::tie(it, inserted) = test.emplace("Stuff", 17);
    EXPECT_TRUE(inserted);
    EXPECT_EQ(test.inverse().at(17), "Stuff");
    EXPECT_EQ(test.inverse().at(789), "Stuff");
    auto [invIt, invInserted] = test.inverse().emplace(1, "Test");
    EXPECT_TRUE(invInserted);
    EXPECT_EQ(test.inverse().at(1), "Test");
    EXPECT_EQ(test.erase("Stuff"), 2);
    EXPECT_FALSE(test.contains("Stuff"));
}

TEST(BidirectionalMap, MultiMapErase)
{
    bimap<std::string, int, std::multimap> test{
        {"Test", 123}, {"Test", 456}, {"Test", 789}, {"One", 1}};
    test.erase("One");
    EXPECT_FALSE(test.contains("One"));
    EXPECT_FALSE(test.inverse().contains(1));
    test.inverse().erase(789);
    EXPECT_EQ(test.inverse().at(123), "Test");
    EXPECT_EQ(test.inverse().at(456), "Test");
    auto [curr, end] = test.equal_range("Test");
    ASSERT_NE(curr, end);
    EXPECT_EQ(curr->second, 123);
    ++curr;
    EXPECT_EQ(curr->second, 456);
    EXPECT_EQ(++curr, end);
}

TEST(BidirectionalMap, MultiMapEquality)
{
    bimap<std::string, int, std::multimap> test{
        {"Test", 123}, {"NewItem", 456}, {"Stuff", 789}};
    auto copy = test;
    EXPECT_EQ(copy, test);
    test.emplace("Test", 17);
    test.emplace("Test", 1337);
    copy.emplace("Test", 1337);
    copy.emplace("Test", 17);
    EXPECT_NE(copy, test);
    test.inverse().erase(17);
    copy.inverse().erase(17);
    EXPECT_EQ(copy, test);
}

TEST(BidirectionalMap, RangesOrdered)
{
    bimap<std::string, int, std::multimap, std::map> test{
        {"Test", 123}, {"NewItem", 456}, {"Test", 789}};
    auto [curr, last] = test.equal_range("Test");
    EXPECT_EQ(curr, test.lower_bound("Test"));
    EXPECT_EQ(last, test.upper_bound("Test"));
    ASSERT_NE(curr, last);
    checkIterator(curr, "Test", 123);
    ASSERT_NE(++curr, last);
    checkIterator(curr, "Test", 789);
    EXPECT_EQ(++curr, last);
    auto [invCurr, invLast] = test.inverse().equal_range(456);
    EXPECT_EQ(invCurr, test.inverse().lower_bound(456));
    EXPECT_EQ(invLast, test.inverse().upper_bound(456));
    ASSERT_NE(invCurr, invLast);
    checkIterator(invCurr, 456, "NewItem");
    EXPECT_EQ(++invCurr, invLast);
}

TEST(BidirectionalMap, RangesUnordered)
{
    bimap<std::string, int, std::unordered_multimap, std::unordered_map> test{
        {"Test", 123}, {"NewItem", 456}, {"Test", 789}};
    std::unordered_set lookup = {123, 789};
    auto [curr, last] = test.equal_range("Test");
    ASSERT_NE(curr, last);
    EXPECT_EQ(curr->first, "Test");
    EXPECT_NE(lookup.find(curr->second), lookup.end());
    lookup.erase(curr->second);
    ASSERT_NE(++curr, last);
    EXPECT_EQ(curr->first, "Test");
    EXPECT_NE(lookup.find(curr->second), lookup.end());
    lookup.erase(curr->second);
    EXPECT_EQ(++curr, last);
    EXPECT_TRUE(lookup.empty());
    auto [invCurr, invLast] = test.inverse().equal_range(456);
    ASSERT_NE(invCurr, invLast);
    checkIterator(invCurr, 456, "NewItem");
    EXPECT_EQ(++invCurr, invLast);
}
