#pragma once

#include <atomic>
#include <condition_variable>
#include <mutex>
#include <queue>
#include <stdexcept>
#include <type_traits>

namespace tl {

// An iterator used with Channel that blocks the current thread, waiting to
// fetch elements from the Channel.
template <typename Channel>
class blocking_iterator
{
public:
    using value_type = typename Channel::value_type;
    using reference = typename Channel::value_type&;

    explicit blocking_iterator(Channel& channel) : channel_{channel} {}

    blocking_iterator<Channel> operator++() const noexcept { return *this; }

    value_type operator*() const
    {
        value_type value{};
        channel_ >> value;

        return value;
    }

    bool operator!=(blocking_iterator<Channel>) const
    {
        std::unique_lock<std::mutex> lock{channel_.mtx_};
        channel_.waitBeforeRead(lock);

        return !(channel_.closed() && channel_.empty());
    }

private:
    Channel& channel_;
};

} // namespace tl

template <typename T>
struct std::iterator_traits<tl::blocking_iterator<T>>
{
    using value_type = typename tl::blocking_iterator<T>::value_type;
    using reference = typename tl::blocking_iterator<T>::reference;
    using iterator_category = std::output_iterator_tag;
};

namespace tl {

class ClosedChannelError : public std::runtime_error
{
public:
    using std::runtime_error::runtime_error;
};

// A Thread-safe container for sharing data between threads
template <typename T>
class Channel
{
public:
    using value_type = T;
    using iterator = blocking_iterator<Channel<T>>;
    using size_type = std::size_t;

    constexpr Channel() = default;
    explicit constexpr Channel(size_type capacity);
    Channel(const Channel&) = delete;
    Channel& operator=(const Channel&) = delete;
    Channel(Channel&&) = delete;
    Channel& operator=(Channel&&) = delete;
    virtual ~Channel() = default;

    /// Actions
    // Put element into Channel
    template <typename Type>
    friend Channel<typename std::decay_t<Type>>& operator<<(
        Channel<typename std::decay_t<Type>>&, Type&&);

    // Pop element for Channel
    template <typename Type>
    friend Channel<Type>& operator>>(Channel<Type>&, Type&);

    [[nodiscard]] inline constexpr size_type size() const noexcept;
    [[nodiscard]] inline constexpr bool empty() const noexcept;

    inline void close() noexcept;
    inline void resume();
    [[nodiscard]] inline bool closed() const noexcept;

    iterator begin() noexcept;
    iterator end() noexcept;

private:
    inline void waitBeforeRead(std::unique_lock<std::mutex>&);
    inline void waitBeforeWrite(std::unique_lock<std::mutex>&);

private:
    const size_type capacity_{0};
    std::queue<T> queue_;
    std::atomic<std::size_t> size_{0};
    std::mutex mtx_;
    std::condition_variable condition_;
    std::atomic<bool> closed_{false};

    friend class blocking_iterator<Channel>;
};

} // namespace tl

#include "channel.impl.hpp"
