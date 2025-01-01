#include <algorithm>

#include <gtest/gtest.h>

#include <tCore/Channel>

using namespace tl;

TEST(Channel, Traits)
{
    using type = int;
    using channel = Channel<type>;
    EXPECT_TRUE((std::is_same_v<channel::value_type, type>));

    using iterator = blocking_iterator<channel>;
    EXPECT_TRUE((std::is_same_v<channel::iterator, iterator>));
    EXPECT_TRUE((std::is_same_v<channel::size_type, std::size_t>));
}

TEST(Channel, PushAndFetch)
{
    Channel<int> channel;

    int in = 1;
    channel << in;

    const int cin = 3;
    channel << cin;

    channel << 2 << 4;

    int out = 0;

    channel >> out;
    EXPECT_EQ(1, out);

    channel >> out;
    EXPECT_EQ(3, out);

    channel >> out;
    EXPECT_EQ(2, out);

    channel >> out;
    EXPECT_EQ(4, out);
}

TEST(Channel, PushAndFetchMultiple)
{
    Channel<int> channel;

    int a = 1;
    const int b = 3;
    channel << a << 2 << b << std::move(a);

    int out = 0;
    int out2 = 0;

    channel >> out;
    EXPECT_EQ(1, out);

    channel >> out;
    EXPECT_EQ(2, out);

    channel >> out >> out2;
    EXPECT_EQ(3, out);
    EXPECT_EQ(1, out2);
}

TEST(Channel, PushByMoveAndFetch)
{
    Channel<std::string> channel;

    std::string in{"abc"};
    channel << std::move(in);

    channel << std::string{"def"};

    std::string out{};
    channel >> out;
    EXPECT_EQ("abc", out);

    channel >> out;
    EXPECT_EQ("def", out);
}

TEST(Channel, Size)
{
    Channel<int> channel;
    EXPECT_EQ(0, channel.size());

    int in = 1;
    channel << in;
    EXPECT_EQ(1, channel.size());

    channel >> in;
    EXPECT_EQ(0, channel.size());
}

TEST(Channel, Empty)
{
    Channel<int> channel;
    EXPECT_TRUE(channel.empty());

    int in = 1;
    channel << in;
    EXPECT_FALSE(channel.empty());

    channel >> in;
    EXPECT_TRUE(channel.empty());
}

TEST(Channel, Close)
{
    Channel<int> channel;
    EXPECT_FALSE(channel.closed());

    int in = 1;
    channel << in;

    channel.close();
    EXPECT_TRUE(channel.closed());

    int out = 0;
    channel >> out;
    EXPECT_EQ(1, out);
    EXPECT_NO_THROW(channel >> out);

    EXPECT_THROW(channel << in, ClosedChannelError);
    EXPECT_THROW(channel << std::move(in), ClosedChannelError);
}

TEST(Channel, Iterator)
{
    Channel<int> channel;

    channel << 1;

    for (auto it = channel.begin(); it != channel.end();) {
        EXPECT_EQ(1, *it);
        break;
    }
}

TEST(Channel, Multithreading)
{
    constexpr int numbers = 10000;
    constexpr std::int64_t expected = 50005000;
    constexpr std::size_t kThreadsToReadFrom = 100;

    Channel<int> channel{10};

    std::mutex mtx_read{};
    std::condition_variable cond_read{};
    bool ready_to_read{};
    std::atomic<std::int64_t> count_numbers{};
    std::atomic<std::int64_t> sum_numbers{};

    std::mutex mtx_wait{};
    std::condition_variable cond_wait{};
    std::atomic<std::size_t> wait_counter{kThreadsToReadFrom};

    auto worker = [&] {
        // Wait until there is data on the channel
        std::unique_lock<std::mutex> lock{mtx_read};
        cond_read.wait(lock, [&ready_to_read] { return ready_to_read; });

        // Read until all items have been read from the channel
        while (count_numbers < numbers) {
            int out{};
            channel >> out;

            sum_numbers += out;
            ++count_numbers;
        }
        --wait_counter;
        cond_wait.notify_one();
    };

    std::vector<std::thread> threads{};
    for (std::size_t i = 0U; i < kThreadsToReadFrom; ++i) {
        threads.emplace_back(std::thread{worker});
    }

    // Send numbers to channel
    for (int i = 1; i <= numbers; ++i) {
        channel << i;

        // Notify threads than then can start reading
        if (!ready_to_read) {
            ready_to_read = true;
            cond_read.notify_all();
        }
    }

    // Wait until all items have been read
    std::unique_lock<std::mutex> lock{mtx_wait};
    cond_wait.wait(lock,
                   [&wait_counter]() { return wait_counter.load() == 0; });

    std::ranges::for_each(threads, [](auto& thread) { thread.join(); });

    EXPECT_EQ(expected, sum_numbers);
}

TEST(ChannelIterator, Traits)
{
    using type = int;
    using iterator = blocking_iterator<Channel<type>>;
    EXPECT_TRUE((std::is_same_v<iterator::value_type, type>));

    using iterator_traits = std::iterator_traits<iterator>;
    EXPECT_TRUE((std::is_same_v<iterator_traits::value_type, type>));
    EXPECT_TRUE((std::is_same_v<iterator_traits::iterator_category,
                                std::output_iterator_tag>));
}

TEST(ChannelIterator, Dereference)
{
    Channel<int> channel;
    blocking_iterator<Channel<int>> it{channel};

    int in = 1;
    channel << in;
    in = 2;
    channel << in;

    EXPECT_EQ(1, *it);
    EXPECT_EQ(2, *it);
}

TEST(ChannelIterator, NotEqualStop)
{
    Channel<int> channel;
    blocking_iterator<Channel<int>> it{channel};

    channel.close();

    EXPECT_FALSE(it != it);
}

TEST(ChannelIterator, NotEqualContinue)
{
    Channel<int> channel;
    blocking_iterator<Channel<int>> it{channel};

    channel << 1;

    EXPECT_TRUE(it != it);
}
